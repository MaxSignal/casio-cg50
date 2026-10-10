package main

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"os"
)

// SH7305 address space. P0/P1/P2 alias phys = vaddr & 0x1FFFFFFF; ROM (OS image)
// at phys 0, DRAM at 0x0C000000, on-chip RAM at 0xFD800000; MMIO delegated to the bus.
// Faithful port of emu/memory.py.

const (
	FlashSize   = 0x08000000 // NOR flash window; image is a prefix, rest reads 0xFF
	FlashMutTop = 0x02000000 // mutable/writable flash extent (32MB): OS image + fls0 storage tail
	IlramBase   = 0xFD800000
	IlramSize   = 0x00010000 // 64 KB
	OcramBase   = 0xFE200000 // on-chip RAM the OS uses for kernel lists/structs (swept 0xFE380000.. etc)
	OcramSize   = 0x00200000 // 2 MB: covers 0xFE224000, 0xFE2FFC24, 0xFE380000-8BFFC, 0xFE3C0000, 0xFE3FFD00
	// X/Y memory (0xE5007000/0xE5017000, gint's .xyram at 0xE500E000) and the IL memory at
	// 0xE5200000 where gint relocates its interrupt callback; the OS never touches them.
	XyramBase = 0xE5000000
	XyramSize = 0x00210000
)

// DRAM placement. The fx-CG50 has 8 MB at 0x0C000000; the fx-CG10/20 (Prizm) has 2 MB at
// 0x08000000 (selected by the bare-metal Linux runner, linux_boot.go).
var (
	DramBase uint32 = 0x0C000000
	DramSize uint32 = 0x00800000
)

// NOR-flash command-state-machine states (AMD/Spansion JEDEC command set). The fls0
// FTL programs/erases flash via these; the boot reads code from the same array.
const (
	fIdle = iota
	fUnlock1
	fUnlock2
	fProgram
	fErase1
	fErase2
	fErase3
	fBufCount
	fBufData
)

type fword struct {
	addr, size, val uint32
}

type Memory struct {
	rom     []byte
	romSize uint32
	flash   []byte // mutable NOR array [0, FlashMutTop): image copy then 0xFF; program/erase mutate it
	dram    []byte
	xyram   []byte // X/Y + IL on-chip memories at 0xE5000000
	ilram   []byte
	ocram   []byte // on-chip RAM at OcramBase
	mmio    *MMIOBus
	mmu     *ccn // MMU (mmu.go); translates P0/U0/P3 when MMUCR.AT=1
	mmuAt   bool // == mmu.at, kept by the MMU (hot-path flag)
	trace   bool
	wpages  map[uint32]int // if non-nil, histogram DRAM write target by 32KB page
	fwrites map[uint32]int // if non-nil, histogram FLASH write target by 32KB page
	fwLog   int            // remaining flash-write detail lines to print

	// DRAM read-watch (investigation only; nil/zero = disabled, so goldens/normal
	// runs are unaffected). When rdPC != nil, any DRAM read with phys in [rdLo,rdHi)
	// is attributed to the reading instruction (cpu.pc, which step() has already
	// advanced by +2). Used to find the operand-fetch PC of the BCD evaluator.
	cpu   *CPU
	rdLo  uint32
	rdHi  uint32
	rdPC  map[uint32]int
	rdLog int

	// DRAM write-watch (investigation only; nil = disabled). Called for every DRAM
	// write with phys in [wrLo,wrHi) — used to catch the PC that draws into VRAM.
	wrLo, wrHi  uint32
	wrHook      func(phys, size, val uint32)
	mmioHook    func(va, size, val uint32) // investigation: every MMIO write (nil = off)
	ubcOps      bool                       // the UBC watches operand accesses (ubc.go stepUBC)
	flashRdHook func(phys, size uint32)    // investigation: data reads from flash (nil = off)
	// accHook (investigation only; nil = off): every access to DRAM (addr = physical address),
	// to the on-chip memories and MMIO at 0xE0000000.. (addr = P4 address), DRAM instruction
	// fetches and DMA source reads. Used by ramfree_probe_test.go to map unused RAM.
	accHook func(addr, size uint32, write bool)

	// NOR command state machine
	fcmd   int
	bufRem int     // remaining buffered-program data words to collect
	bufW   []fword // buffered-program collected words

	// Sector erase in progress: the next eraseBusy data reads of the erasing sector return the
	// embedded-algorithm status instead of array data (see flashStatus).
	eraseBusy int
	eraseSect uint32
	eraseTog  uint32
}

// Flash geometry: the OS erases 128 KB sectors (its erase routine in IL RAM masks the address
// with 0xFFFE0000; S29GL-S style uniform sectors).
const (
	flashSector = 0x20000
	// eraseStatusReads is how many status reads a sector erase lasts. The OS's erase routine
	// (IL RAM 0xFD800BB6) checks right after the command that the erase has started (a read with
	// DQ7=0, then one with DQ3=1, 0xFD800CA8) and then polls until DQ7 reads 1 (0xFD800D0E); an
	// erase that is already over at the first read is reported as failed (-3).
	eraseStatusReads = 4
)

// flashStatus is a status read during a sector erase: DQ7=0 (busy), DQ6 and DQ2 toggling on
// every read, DQ3=1 (erase timer expired: the erase has started), DQ5=0 (no timeout).
func (m *Memory) flashStatus(size uint32) uint32 {
	m.eraseBusy--
	m.eraseTog ^= 0x44
	st := 0x08 | m.eraseTog
	return st * (0x01010101 >> ((4 - size) * 8))
}

func NewMemory(osImage []byte, mmio *MMIOBus) *Memory {
	flash := make([]byte, FlashMutTop)
	n := copy(flash, osImage)
	for i := n; i < len(flash); i++ {
		flash[i] = 0xFF // erased NOR
	}
	var mmu *ccn
	if mmio != nil {
		mmu = mmio.ccn
	}
	m := &Memory{
		mmu:     mmu,
		rom:     osImage,
		romSize: uint32(len(osImage)),
		flash:   flash,
		dram:    make([]byte, DramSize),
		xyram:   make([]byte, XyramSize),
		ilram:   make([]byte, IlramSize),
		ocram:   make([]byte, OcramSize),
		mmio:    mmio,
	}
	if mmu != nil {
		mmu.mem = m
	}
	return m
}

// Read returns a big-endian value of 1/2/4 bytes at virtual address va.
// translated reports whether va goes through the MMU (P0/U0 or P3 with MMUCR.AT=1).
func (m *Memory) translated(va uint32) bool {
	return m.mmuAt && (va < 0x80000000 || va-0xC0000000 < 0x20000000)
}

func (m *Memory) priv() bool { return m.cpu == nil || m.cpu.sr&srMD != 0 }

func (m *Memory) Read(va, size uint32) uint32 {
	if m.ubcOps {
		return m.ubcRead(va, size)
	}
	if m.translated(va) {
		va = 0xA0000000 | m.mmu.translate(va, false, m.priv())
	}
	if va >= 0xE0000000 {
		if m.accHook != nil {
			m.accHook(va, size, false)
		}
		if va >= IlramBase && va < IlramBase+IlramSize {
			return beRead(m.ilram, va-IlramBase, size)
		}
		if va-XyramBase < XyramSize {
			return beRead(m.xyram, va-XyramBase, size)
		}
		if va >= OcramBase && va < OcramBase+OcramSize {
			return beRead(m.ocram, va-OcramBase, size)
		}
		return m.mmio.Read(va, size) // control regs (0xFF.., 0xFE.., store queues)
	}
	if mphys := va & 0x1FFFFFFF; mphys >= DramBase && mphys < DramBase+DramSize {
		// DRAM via ANY mirror, incl. the uncached VRAM mirror 0xAC000000 (P2). Must come
		// before the 0xA4..0xC0 MMIO range below, else VRAM draws are dropped as MMIO.
		if m.rdPC != nil && mphys >= m.rdLo && mphys < m.rdHi {
			m.rdPC[m.cpu.pc]++
			if m.rdLog > 0 {
				v := beRead(m.dram, mphys-DramBase, size)
				fmt.Printf("  [rd] phys=0x%08x sz=%d pc=0x%08x (instr~0x%08x) val=0x%x\n",
					mphys, size, m.cpu.pc, m.cpu.pc-2, v)
				m.rdLog--
			}
		}
		if m.accHook != nil {
			m.accHook(mphys, size, false)
		}
		return beRead(m.dram, mphys-DramBase, size)
	}
	if va >= 0xA4000000 && va < 0xC0000000 {
		return m.mmio.Read(va, size) // on-chip periph + area-5 LCD as MMIO
	}
	phys := va & 0x1FFFFFFF
	if phys < FlashSize {
		if m.flashRdHook != nil {
			m.flashRdHook(phys, size)
		}
		if phys < FlashMutTop {
			if m.eraseBusy > 0 && phys&^(flashSector-1) == m.eraseSect {
				return m.flashStatus(size)
			}
			return beRead(m.flash, phys, size) // array-read mode (mutable NOR)
		}
		return (uint32(1) << (size * 8)) - 1 // beyond mutable extent: erased 0xFF
	}
	if phys >= DramBase && phys < DramBase+DramSize {
		return beRead(m.dram, phys-DramBase, size)
	}
	panic(memFault("read", va, size))
}

func (m *Memory) Write(va, size, val uint32) {
	if m.ubcOps {
		m.ubcWrite(va, size, val)
		return
	}
	if m.translated(va) {
		va = 0xA0000000 | m.mmu.translate(va, true, m.priv())
	}
	mask := (uint32(1) << (size * 8)) - 1
	if size == 4 {
		mask = 0xFFFFFFFF
	}
	val &= mask
	if va >= 0xE0000000 {
		if m.accHook != nil {
			m.accHook(va, size, true)
		}
		if va >= IlramBase && va < IlramBase+IlramSize {
			beWrite(m.ilram, va-IlramBase, size, val)
			return
		}
		if va-XyramBase < XyramSize {
			beWrite(m.xyram, va-XyramBase, size, val)
			return
		}
		if va >= OcramBase && va < OcramBase+OcramSize {
			beWrite(m.ocram, va-OcramBase, size, val)
			return
		}
		if m.mmioHook != nil {
			m.mmioHook(va, size, val)
		}
		m.mmio.Write(va, size, val)
		return
	}
	if mphys := va & 0x1FFFFFFF; mphys >= DramBase && mphys < DramBase+DramSize {
		// DRAM via ANY mirror, incl. uncached VRAM 0xAC000000 — must precede the MMIO range.
		if m.wpages != nil {
			m.wpages[(mphys-DramBase)&^0x7FFF]++
		}
		if m.wrHook != nil && mphys >= m.wrLo && mphys < m.wrHi {
			m.wrHook(mphys, size, val)
		}
		if m.accHook != nil {
			m.accHook(mphys, size, true)
		}
		beWrite(m.dram, mphys-DramBase, size, val)
		return
	}
	if va >= 0xA4000000 && va < 0xC0000000 {
		if m.mmioHook != nil {
			m.mmioHook(va, size, val)
		}
		m.mmio.Write(va, size, val)
		return
	}
	phys := va & 0x1FFFFFFF
	if phys < FlashSize {
		if m.fwrites != nil {
			m.fwrites[phys&^0x7FFF]++
			if m.fwLog > 0 {
				fmt.Printf("  [flashwr] va=0x%08x phys=0x%08x size=%d val=0x%x st=%d\n", va, phys, size, val, m.fcmd)
				m.fwLog--
			}
		}
		m.flashCmd(phys, size, val)
		return
	}
	if phys >= DramBase && phys < DramBase+DramSize {
		if m.wpages != nil {
			m.wpages[(phys-DramBase)&^0x7FFF]++
		}
		beWrite(m.dram, phys-DramBase, size, val)
		return
	}
	panic(memFault("write", va, size))
}

// flashCmd drives the NOR command state machine. Only PROGRAM/ERASE mutate the array;
// unlock and ID/CFI/reset commands change state only (so code-fetch reads stay valid).
func (m *Memory) flashCmd(phys, size, val uint32) {
	a12 := phys & 0xFFF
	c := val & 0xFF
	switch m.fcmd {
	case fIdle:
		if a12 == 0xAAA && c == 0xAA {
			m.fcmd = fUnlock1
		}
	case fUnlock1:
		if a12 == 0x554 && c == 0x55 {
			m.fcmd = fUnlock2
		} else {
			m.fcmd = fIdle
		}
	case fUnlock2:
		switch {
		case a12 == 0xAAA && c == 0xA0: // program
			m.fcmd = fProgram
		case a12 == 0xAAA && c == 0x80: // erase setup
			m.fcmd = fErase1
		case c == 0x25: // write-to-buffer (addr = sector); count comes next
			m.bufW = m.bufW[:0]
			m.bufRem = -1
			m.fcmd = fBufCount
		default: // 0x90 autoselect, 0x98 CFI, 0xF0 reset, etc — no array change
			m.fcmd = fIdle
		}
	case fProgram:
		m.flashProgram(phys, size, val)
		m.fcmd = fIdle
	case fErase1:
		if a12 == 0xAAA && c == 0xAA {
			m.fcmd = fErase2
		} else {
			m.fcmd = fIdle
		}
	case fErase2:
		if a12 == 0x554 && c == 0x55 {
			m.fcmd = fErase3
		} else {
			m.fcmd = fIdle
		}
	case fErase3:
		if c == 0x10 { // chip erase
			for i := range m.flash {
				m.flash[i] = 0xFF
			}
		} else if c == 0x30 { // sector erase (128 KB) containing phys
			s := phys &^ (flashSector - 1)
			for i := s; i < s+flashSector && i < FlashMutTop; i++ {
				m.flash[i] = 0xFF
			}
			m.eraseBusy, m.eraseSect = eraseStatusReads, s
		}
		m.fcmd = fIdle
	case fBufCount:
		m.bufRem = int(val&0xFFFF) + 1 // word count - 1 was written
		m.fcmd = fBufData
	case fBufData:
		if m.bufRem > 0 {
			m.bufW = append(m.bufW, fword{phys, size, val})
			m.bufRem--
		} else { // confirm (0x29) — commit all buffered words
			for _, w := range m.bufW {
				m.flashProgram(w.addr, w.size, w.val)
			}
			m.fcmd = fIdle
		}
	}
}

// flashProgram performs a NOR word/byte program: bits can only go 1->0 (AND).
func (m *Memory) flashProgram(phys, size, val uint32) {
	if phys+size > FlashMutTop {
		return
	}
	for i := uint32(0); i < size; i++ {
		b := byte(val >> (8 * (size - 1 - i)))
		m.flash[phys+i] &= b
	}
}

// ---- fls0 persistence -------------------------------------------------------
// The fx-CG50 keeps its settings + first-boot-complete state in the NOR-flash fls0
// filesystem (phys 0x01000000+, the storage tail past the 16MB OS image, which boots
// blank=0xFF -> first-boot setup runs every cold start). Snapshotting the flash pages the
// OS wrote (and reloading them next boot) makes the emulator resume at the MAIN MENU like a
// real, already-set-up calculator. We persist a DELTA vs the fresh baseline (OS image prefix
// then 0xFF erased), so the file holds only the fls0 data, not the copyrighted OS image.

const flashPersistPage = 0x1000 // 4KB granularity (NOR erase is 64KB; finer = smaller file)

// flsBaseline is the byte the freshly-initialised mutable flash holds at off.
func (m *Memory) flsBaseline(off int) byte {
	if off < len(m.rom) {
		return m.rom[off]
	}
	return 0xFF // erased NOR tail
}

// flashDeltaBytes serialises only the flash pages that differ from the baseline.
// Format: "FLS1" magic, then repeated {u32 BE page offset, flashPersistPage bytes}.
func (m *Memory) flashDeltaBytes() []byte {
	var buf bytes.Buffer
	buf.WriteString("FLS1")
	for off := 0; off+flashPersistPage <= len(m.flash); off += flashPersistPage {
		diff := false
		for i := off; i < off+flashPersistPage; i++ {
			if m.flash[i] != m.flsBaseline(i) {
				diff = true
				break
			}
		}
		if diff {
			var hdr [4]byte
			binary.BigEndian.PutUint32(hdr[:], uint32(off))
			buf.Write(hdr[:])
			buf.Write(m.flash[off : off+flashPersistPage])
		}
	}
	return buf.Bytes()
}

// applyFlashDelta overlays a serialised delta onto the (baseline-initialised) mutable flash.
func (m *Memory) applyFlashDelta(data []byte) (int, error) {
	if len(data) < 4 || string(data[:4]) != "FLS1" {
		return 0, fmt.Errorf("flash delta: bad magic")
	}
	p, pages := 4, 0
	for p < len(data) {
		if p+4+flashPersistPage > len(data) {
			return pages, fmt.Errorf("flash delta: truncated record at byte %d", p)
		}
		off := int(binary.BigEndian.Uint32(data[p : p+4]))
		p += 4
		if off < 0 || off+flashPersistPage > len(m.flash) {
			return pages, fmt.Errorf("flash delta: bad page offset 0x%x", off)
		}
		copy(m.flash[off:off+flashPersistPage], data[p:p+flashPersistPage])
		p += flashPersistPage
		pages++
	}
	return pages, nil
}

// SaveFlashDelta / LoadFlashDelta persist the flash delta to a standalone file.
func (m *Memory) SaveFlashDelta(path string) (int, error) {
	b := m.flashDeltaBytes()
	pages := (len(b) - 4) / (4 + flashPersistPage)
	return pages, os.WriteFile(path, b, 0644)
}

func (m *Memory) LoadFlashDelta(path string) (int, error) {
	data, err := os.ReadFile(path)
	if err != nil {
		return 0, err
	}
	return m.applyFlashDelta(data)
}

// span returns n bytes starting at va for a DMA source: a direct view when the range lies in
// DRAM (the VRAM push), else a copy read through the bus.
func (m *Memory) span(va, n uint32) []byte {
	if p := va & 0x1FFFFFFF; p >= DramBase && p+n <= DramBase+DramSize {
		if m.accHook != nil {
			m.accHook(p, n, false)
		}
		return m.dram[p-DramBase : p-DramBase+n]
	}
	if va-XyramBase < XyramSize && va-XyramBase+n <= XyramSize {
		if m.accHook != nil {
			m.accHook(va, n, false)
		}
		return m.xyram[va-XyramBase : va-XyramBase+n]
	}
	b := make([]byte, n) // DMA addresses are physical: read through the uncached P2 alias
	for i := uint32(0); i < n; i++ {
		b[i] = byte(m.Read(0xA0000000|(va+i)&0x1FFFFFFF, 1))
	}
	return b
}

// fetch16 reads the instruction halfword at pc. Untranslated code in flash or DRAM (nearly all
// execution) is read straight from the backing slice; anything else (MMU on, MMIO windows,
// P4) takes the general Read path. Same result either way — this is only the hot path.
func (m *Memory) fetch16(pc uint32) uint32 {
	if !m.mmuAt && pc < 0xA4000000 {
		p := pc & 0x1FFFFFFF
		if p < FlashMutTop {
			return uint32(m.flash[p])<<8 | uint32(m.flash[p+1])
		}
		if p-DramBase < DramSize {
			if m.accHook != nil {
				m.accHook(p, 2, false)
			}
			return uint32(m.dram[p-DramBase])<<8 | uint32(m.dram[p-DramBase+1])
		}
	}
	if m.ubcOps { // a delay slot fetched inside a watched instruction: not an operand access
		m.ubcOps = false
		v := m.Read(pc, 2)
		m.ubcOps = true
		return v
	}
	return m.Read(pc, 2)
}

func (m *Memory) R8(va uint32) uint32  { return m.Read(va, 1) }
func (m *Memory) R16(va uint32) uint32 { return m.Read(va, 2) }
func (m *Memory) R32(va uint32) uint32 { return m.Read(va, 4) }
func (m *Memory) W8(va, v uint32)      { m.Write(va, 1, v) }
func (m *Memory) W16(va, v uint32)     { m.Write(va, 2, v) }
func (m *Memory) W32(va, v uint32)     { m.Write(va, 4, v) }

// ubcRead / ubcWrite: an operand access of the instruction being executed while the UBC
// watches them (ubcOps, set only inside stepUBC). Accesses nested in it (an MMIO write that
// starts a DMA transfer, say) are not the CPU's and are not seen.
func (m *Memory) ubcRead(va, size uint32) uint32 {
	m.ubcOps = false
	v := m.Read(va, size)
	m.ubcOps = true
	m.cpu.ubc.operand(va, size, v, false)
	return v
}

func (m *Memory) ubcWrite(va, size, val uint32) {
	m.ubcOps = false
	m.cpu.ubc.operand(va, size, val, true)
	m.Write(va, size, val)
	m.ubcOps = true
}

// big-endian helpers
func beRead(buf []byte, off, size uint32) uint32 {
	var v uint32
	for i := uint32(0); i < size; i++ {
		v = (v << 8) | uint32(buf[off+i])
	}
	return v
}

func beReadClamped(buf []byte, off, size, limit uint32) uint32 {
	var v uint32
	for i := uint32(0); i < size; i++ {
		var b byte = 0xFF
		if off+i < limit {
			b = buf[off+i]
		}
		v = (v << 8) | uint32(b)
	}
	return v
}

func beWrite(buf []byte, off, size, val uint32) {
	for i := uint32(0); i < size; i++ {
		buf[off+size-1-i] = byte(val >> (8 * i))
	}
}

type MemFaultError struct{ msg string }

func (e MemFaultError) Error() string { return e.msg }

func memFault(op string, va, size uint32) MemFaultError {
	return MemFaultError{msg: fmt.Sprintf("%s unmapped 0x%08x (size %d)", op, va, size)}
}
