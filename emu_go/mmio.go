package main

import (
	"fmt"
	"maps"
)

// SH7305 MMIO bus + peripheral stubs. Faithful port of emu/mmio.py.
// Stubs return values that satisfy the OS poll loops (PLL ready, DMA done,
// key released) so boot/run progresses.

type region interface {
	contains(va uint32) bool
	name() string
	read(va, size uint32) uint32
	write(va, size, val uint32)
}

// base: a logged register window backed by a map (offset -> last written value).
type base struct {
	nm   string
	bs   uint32
	sz   uint32
	regs map[uint32]uint32
}

func newBase(nm string, bs, sz uint32) base {
	return base{nm: nm, bs: bs, sz: sz, regs: map[uint32]uint32{}}
}
func (b *base) contains(va uint32) bool { return va >= b.bs && va < b.bs+b.sz }
func (b *base) name() string            { return b.nm }
func (b *base) read(va, size uint32) uint32 {
	return b.regs[va-b.bs]
}
func (b *base) write(va, size, val uint32) { b.regs[va-b.bs] = val }

// txPin: PFC port J data (0xA4050130), passed through to the PFC, counting the changes of
// bit 2, the transmit pin of the 3-pin port that Linux toggles to make tones.
type txPin struct {
	pfc     *base
	toggles uint64
}

func (t *txPin) contains(va uint32) bool    { return va == 0xA4050130 }
func (t *txPin) name() string               { return "PFC" }
func (t *txPin) read(va, size uint32) uint32 { return t.pfc.read(va, size) }
func (t *txPin) write(va, size, val uint32) {
	if (t.pfc.regs[0x130]^val)&4 != 0 {
		t.toggles++
	}
	t.pfc.write(va, size, val)
}

// CPG: boot PLL routine spins on 'ready' bit0 at +0x60 -> return 0 so it exits.
type cpg struct{ base }

func (c *cpg) read(va, size uint32) uint32 {
	off := va - c.bs
	if off == 0x60 {
		return 0
	}
	return c.regs[off]
}

// DMAC (SH7724 layout): channels 0-3 at +0x20,+0x30,+0x40,+0x50, DMAOR at +0x60, channels
// 4-5 at +0x70,+0x80; per channel SAR +0, DAR +4, TCR +8, CHCR +0xC. The OS pushes VRAM to
// the LCD (area 5, DAR 0x14000000) on channel 0 (+0x20); starting such a channel (CHCR.DE=1)
// is the moment the real screen changes, so onLCDPush (if set) is told the source address
// then. Transfer-end interrupts: DEI0-3 0x800/0x820/0x840/0x860, DEI4-5 0xb80/0xba0.
type dmac struct {
	base
	bus       *MMIOBus
	onLCDPush func(sar uint32)
}

// dmaUnit is the transfer-unit size in bytes for CHCR.TS (TS[1:0]=bits 4:3, TS[3:2]=bits
// 21:20): the OS's VRAM push uses TS=0100 (32-byte units, TCR 0x1440 = one 384x216 frame).
func dmaUnit(chcr uint32) uint32 {
	switch (chcr>>3)&3 | (chcr>>20&3)<<2 {
	case 0:
		return 1
	case 1:
		return 2
	case 2:
		return 4
	case 3:
		return 16
	}
	return 32
}

// CHCR.TE (bit1) is tracked once software has written the channel: a transfer starts on a
// write with DE=1 and TE=0 and sets TE when it completes (instantly); writing DE=1 with TE
// still set does nothing, as on the real DMAC — gint's world switch restores every channel
// register it saved, DE and TE included, and must not re-run the last transfer. A channel
// never written reads TE=1 (idle), as it always did.
func (d *dmac) read(va, size uint32) uint32 {
	off := va - d.bs
	if (off & 0xF) == 0xC {
		if v, ok := d.regs[off]; ok {
			return v
		}
		return 0x2
	}
	return d.regs[off]
}

func (d *dmac) write(va, size, val uint32) {
	off := va - d.bs
	d.regs[off] = val
	ch := -1
	switch chOff := off &^ 0xF; {
	case chOff >= 0x20 && chOff < 0x60:
		ch = int(chOff-0x20) >> 4
	case chOff == 0x70:
		ch = 4
	case chOff == 0x80:
		ch = 5
	}
	if (off&0xF) == 0xC && ch >= 0 && val&1 != 0 && val&2 == 0 {
		d.regs[off] = val | 0x2 // completes instantly
		if d.regs[off-0xC+4]&0x1FFFFFFF == 0x14000000 {
			sar := d.regs[off-0xC]
			n := min(d.regs[off-0xC+8]*dmaUnit(val), 1<<20)
			fixed := (val>>12)&3 == 0 // SM=00: the same source unit every time
			if d.bus != nil && d.bus.lcd != nil && d.bus.cpu != nil {
				if fixed {
					// a fill: DrawFrame (0x800561EE) paints the frame around the OS's area by
					// repeating 32 bytes of the frame colour (FUN_800562A2)
					unit := d.bus.cpu.mem.span(sar, dmaUnit(val))
					for i := uint32(0); i < n; i += uint32(len(unit)) {
						d.bus.lcd.dma(unit)
					}
				} else {
					d.bus.lcd.dma(d.bus.cpu.mem.span(sar, n))
				}
			}
			if d.onLCDPush != nil && !fixed {
				d.onLCDPush(sar) // a VRAM frame push
			}
		} else if d.bus != nil && d.bus.cpu != nil {
			// memory-to-memory transfer (gint's dma_memset/dma_memcpy: VRAM clears and
			// copies from a fixed 32-byte pattern in IL memory); the OS only DMAs to the LCD
			d.memTransfer(d.regs[off-0xC], d.regs[off-0xC+4], d.regs[off-0xC+8], dmaUnit(val), val)
		}
		// the transfer completes instantly; CHCR.IE (bit2) asks for the transfer-end
		// interrupt (gint's display driver sleeps on it; the OS never sets IE)
		if val&4 != 0 && d.bus != nil && d.bus.cpu != nil {
			d.bus.raise(d.bus.cpu, dmacINTEVT[ch], 0)
		}
	}
}

// dmaAddr maps a DMA (physical) address to the bus address the memory model serves: the
// on-chip X/Y/IL memories keep their P4 address, everything else goes through the uncached
// P2 alias.
func dmaAddr(a uint32) uint32 {
	if a-XyramBase < XyramSize {
		return a
	}
	return 0xA0000000 | (a & 0x1FFFFFFF)
}

// memTransfer performs a whole DMAC transfer instantly: count units of unit bytes, with the
// CHCR source/destination modes (SM bits 13:12, DM bits 15:14: 0 fixed, 1 increment,
// 2 decrement). Capped at 4 MB.
func (d *dmac) memTransfer(sar, dar, count, unit, chcr uint32) {
	mem := d.bus.cpu.mem
	if count*unit > 1<<22 {
		count = (1 << 22) / unit
	}
	sm, dm := (chcr>>12)&3, (chcr>>14)&3
	buf := make([]byte, unit)
	for i := uint32(0); i < count; i++ {
		for j := uint32(0); j < unit; j++ {
			buf[j] = byte(mem.Read(dmaAddr(sar+j), 1))
		}
		for j := uint32(0); j < unit; j++ {
			mem.Write(dmaAddr(dar+j), 1, uint32(buf[j]))
		}
		switch sm {
		case 1:
			sar += unit
		case 2:
			sar -= unit
		}
		switch dm {
		case 1:
			dar += unit
		case 2:
			dar -= unit
		}
	}
}

// ETMU: one-shot delays poll elapsed/underflow (bit15) at +0x60 -> report elapsed.
type etmu struct{ base }

func (e *etmu) read(va, size uint32) uint32 {
	off := (va - e.bs) & 0xFFFF
	if off == 0x60 {
		return 0x8000
	}
	return e.regs[off]
}

// CCN: MMU/cache control. The reset stub reads the HW model strap at +0x24 (0xFF000024)
// and selects model code: low16==0x0000->0xCA00, ==0x0020->0xCA01, ==0x0A02->0xCA02 (fx-CG50).
// We report 0x0A02 so the OS identifies as fx-CG50 (else fls0_init's verify loop @0x80365418
// never exits: it needs *(0xfd8018d4)==0xca02).
// FreeCounter: monotonic counter; boot delay loops read twice and wait for advance.
type freeCounter struct {
	base
	count uint32
}

func (f *freeCounter) read(va, size uint32) uint32 {
	f.count++
	mask := (uint32(1) << (size * 8)) - 1
	if size == 4 {
		mask = 0xFFFFFFFF
	}
	return f.count & mask
}

// PeriphIRQ (0xA4610000): flag at +0x88 (bits14/15 = timer underflow). Timer sets it,
// ISR acks by clearing. Treat the 0x88..0x8B word as one.
// periphIRQ: the battery ADC block at 0xA4610000 (cont.18r/18s, tools/tickprobe on the real
// calc). +0x82/+0x84 = A/D result, +0x88/+0x8A = control with end flags bits 14/15 (the
// INTEVT 0x560 ISR 0x801ded94 clears them), +0x8C bit15 = start. The OS idle routine 0x802ae87a
// starts a conversion right before every `sleep`; on the real calc a started conversion never
// completes within 2 s while the CPU runs, yet readings do arrive — i.e. it converts while the
// CPU sleeps. There are two ways to start a conversion:
//   - the idle routine's start, +0x8C bit15: completes after 1/adcSleepDiv s of CPU *sleep*
//     since the start (counted in cpu.idle, so scheduled and per-instruction loops agree);
//   - a software start, +0x88 |= 0x2000 (battery monitor 0x801de54a: start routine 0x801de64a
//     then waits, CPU running, for the ISR's event or +0x88 bit15): completes after
//     1/adcSoftDiv s (a normal A/D conversion time), awake or asleep.
//
// Either raises 0x560 once and sets the end flags. Until the OS first starts a conversion, the
// legacy periodic tick (timerPeriod) keeps running — the fresh boot from reset was proven with it.
type periphIRQ struct {
	base
	bus      *MMIOBus
	adcMode  bool   // the OS has started a conversion: 0x560 is conversion-driven from now on
	armed    bool   // an idle-routine conversion is in progress
	doneAt   uint64 // cpu.idle value at which it completes
	swArmed  bool   // a software-started conversion is in progress
	swDoneAt uint64 // cpu.cycles value at which it completes
}

// adcSleepDiv: an idle-routine conversion needs 1/adcSleepDiv s of CPU sleep. The real latency
// is unknown (probing it by sleeping from an add-in reset the calculator); readings arriving at
// most 64 times per idle second is plenty for a battery gauge and costs almost nothing.
// adcSoftDiv: a software-started conversion takes 1/adcSoftDiv s (100 µs).
const (
	adcSleepDiv = 64
	adcSoftDiv  = 10000
)

// adc keys in the save-state's per-region map (beyond the real register offsets)
const (
	adcKeyMode   = 0x10000
	adcKeyArmed  = 0x10001
	adcKeyLeft   = 0x10002 // doneAt - cpu.idle
	adcKeySw     = 0x10003
	adcKeySwLeft = 0x10004 // swDoneAt - cpu.cycles
)

// Battery-voltage ADC reading reported at PERIPH_IRQ +0x82/+0x84. The OS battery
// monitor (FUN_801de54a) averages two samples and buckets the result (>>6) against
// thresholds ~347-475 (FUN_801e6bbc). A 0 reading falls below all thresholds -> level
// 0x12 -> the shell raises a "battery event" every loop and SKIPS drawing the main
// menu, parking in the idle event-pump. Reporting a normal mid-range voltage
// (raw>>6 == 453 -> bucket 2 "normal") lets the menu draw. 0x7140>>6 = 453.
const adcBatteryRaw = 0x7140

func (p *periphIRQ) read(va, size uint32) uint32 {
	off := va - p.bs
	if off == 0x82 || off == 0x84 {
		return adcBatteryRaw
	}
	if off >= 0x88 && off < 0x8C {
		off = 0x88
	}
	return p.regs[off]
}
func (p *periphIRQ) write(va, size, val uint32) {
	off := va - p.bs
	if off == 0x88 && val&0x2000 != 0 && p.bus != nil && p.bus.cpu != nil {
		p.adcMode, p.swArmed = true, true
		p.swDoneAt = p.bus.cpu.cycles + p.bus.instrPerSec/adcSoftDiv
		val &^= 0x2000 // the start bit is a trigger
	}
	if off >= 0x88 && off < 0x8C {
		off = 0x88
	}
	p.regs[off] = val
	if off == 0x8C && p.bus != nil && p.bus.cpu != nil {
		p.armed = val&0x8000 != 0
		if p.armed {
			p.adcMode = true
			p.doneAt = p.bus.cpu.idle + p.bus.instrPerSec/adcSleepDiv
		}
	}
}

// adcTick completes conversions that are due: a software start after its conversion time, an
// idle-routine start once the CPU has slept long enough.
func (p *periphIRQ) adcTick(cpu *CPU) {
	if p.swArmed && cpu.cycles >= p.swDoneAt {
		p.swArmed = false
		p.setTimerFlag()
		p.bus.raise(cpu, TimerINTEVT, TimerLevel)
	}
	if p.armed && cpu.idle >= p.doneAt {
		p.armed = false
		p.regs[0x8C] &^= 0x8000
		p.setTimerFlag()
		p.bus.raise(cpu, TimerINTEVT, TimerLevel)
	}
}
func (p *periphIRQ) setTimerFlag() { p.regs[0x88] |= 0xC000 }

// INTX (0xA4140000): byte +0x24 reports key-scan ready (bit6) so the timer ISR's
// scan poll completes.
type intx struct{ base }

func (x *intx) read(va, size uint32) uint32 {
	if (va-x.bs)&0xFFFF == 0x24 {
		return 0x40
	}
	return x.regs[va-x.bs]
}

// bcdALU: hardware multi-word BCD arithmetic unit @0xA4CB0000 (RE'd cont.18c, command
// set confirmed by on-device probe cont.18e — os/devic_probes/). The Casio number/format
// library drives it for every decimal +/- (FUN_80072e78 etc.); SH4 has no BCD opcodes, so
// this peripheral does the packed-BCD digit math while software does shifts/masks (SHLD).
// Registers (the 4-word block aliases every 0x10; the OS uses the +0x10 alias):
//
//	+0x00 command/status   +0x04 operand A   +0x08 operand B   +0x0C result
//
// Operands are sticky; a command write triggers the op and the result is valid immediately.
// The mantissa is fed one 32-bit word at a time, LSW first. There is a SINGLE shared
// carry/borrow latch (proven on hardware: a sub's borrow-out feeds a following add as
// carry-in). Command decode (16-bit; bit3 ignored so 8..15 mirror 0..7):
//
//	op      = (cmd&1) ? BCD add : BCD sub
//	flag_in = (cmd&4) ? 1 : (cmd&2) ? latch : 0       (forced-1 / latched / forced-0)
//	flag_out = carry (add) or borrow (sub) -> latched for the next op.
//
// So: 0=A-B  1=A+B  2=A-B-flag  3=A+B+flag  4=A-B-1  5=A+B+1 (OS only uses 0..4).
// VALIDATED: with this model the OS formatter renders "98765"/"4.695555556" instead of "0"
// (the unmodelled result reg reading 0 was the root cause of "all results show 0", cont.18c).
type bcdALU struct {
	base
	a, b, result uint32
	flag         uint32 // shared carry/borrow latch (one bit)
}

func bcdAdd(a, b, carry uint32) (uint32, uint32) {
	var res uint32
	for i := uint32(0); i < 8; i++ {
		s := ((a >> (4 * i)) & 0xF) + ((b >> (4 * i)) & 0xF) + carry
		carry = 0
		if s >= 10 {
			s -= 10
			carry = 1
		}
		res |= s << (4 * i)
	}
	return res, carry
}

func bcdSub(a, b, borrow uint32) (uint32, uint32) {
	var res uint32
	for i := uint32(0); i < 8; i++ {
		s := int(((a >> (4 * i)) & 0xF)) - int((b>>(4*i))&0xF) - int(borrow)
		borrow = 0
		if s < 0 {
			s += 10
			borrow = 1
		}
		res |= uint32(s) << (4 * i)
	}
	return res, borrow
}

// compute runs one BCD op for the written command, latching the shared carry/borrow flag.
func (u *bcdALU) compute(cmd uint32) {
	var fin uint32
	switch {
	case cmd&4 != 0:
		fin = 1
	case cmd&2 != 0:
		fin = u.flag
	}
	if cmd&1 != 0 {
		u.result, u.flag = bcdAdd(u.a, u.b, fin)
	} else {
		u.result, u.flag = bcdSub(u.a, u.b, fin)
	}
}

func (u *bcdALU) write(va, size, val uint32) {
	switch (va - u.bs) & 0xF { // block aliases every 0x10
	case 0x4:
		u.a = val
	case 0x8:
		u.b = val
	case 0x0: // command -> compute (bit3 ignored)
		u.compute(val & 0x7)
	default:
		u.regs[(va-u.bs)&0xFFFF] = val
	}
}

func (u *bcdALU) read(va, size uint32) uint32 {
	switch (va - u.bs) & 0xF {
	case 0xC: // result
		return u.result
	case 0x0: // command/status: the shared flag is observable here (cont.18e)
		if u.flag != 0 {
			return 0x10040000
		}
		return 0x00010000
	}
	return u.regs[va-u.bs]
}

// ---- the bus ----
const (
	TimerINTEVT = 0x560
	TimerLevel  = 8
)

type MMIOBus struct {
	regions []region
	// CPUOPM (0xFF2F0000), the SH-4A CPU operation mode register. Only INTMU (bit 3) is
	// modelled: when set, accepting an interrupt also sets SR.IMASK to its level (cpu.go).
	// gint sets it at startup and relies on it; the OS never touches the register.
	cpuopm    *base
	ubc       *ubcUnit // User Break Controller 0xFF200000 (ubc.go)
	periphIRQ *periphIRQ
	etmu2     *etmuCounter
	tmu       *tmu      // TMU0-2 (gtimer.go); ETMU0-5 live in etmu2
	tx        *txPin    // the 3-pin port's transmit pin
	intc      *intcUnit // priority/mask gate for every interrupt source (intc.go)
	cpu       *CPU
	unknown   map[uint32]int
	watchPC   map[uint32]int // PC histogram of readers of watchBase region
	watchBase uint32         // if nonzero, reads in [watchBase, watchBase+0x1000) are attributed to cpu.pc

	timerPeriod uint64
	dirty       bool           // set by every MMIO write: device event times may have moved (Emulator.Step)
	keysc       *keyscUnit     // key-scan unit @0xA44B0000 (keysc.go): the real key path
	dmac        *dmac          // DMA controller; streams LCD-bound transfers into lcd
	lcd         *lcd           // R61524 panel controller: GRAM = what the user sees (lcd.go)
	ccn         *ccn           // MMU/cache control + UTLB (mmu.go)
	wcount      map[string]int // if non-nil: MMIO writes per region (diagnostics)
	rtc         *rtc           // real-time clock: calendar, 64 Hz counter, periodic IRQ (rtc.go)
	instrPerSec uint64         // host throughput; converts cycles to time for RTC/ETMU/KEYSC
	timerNext   uint64
	timerTicks  uint64

	// KEYSC keypress injection: during [kbStart,kbEnd) cycles, reads of the KEYSC
	// region at offset kbReg return kbVal (kbReg<0 => all offsets return kbVal).
	kbStart, kbEnd uint64
	kbReg          int32
	kbVal          uint32

	// scan-protocol capture (scancap mode): when scanCap is set, every access to the
	// KEYSC/KIU/PFC register blocks is recorded so we can see how the OS scans the matrix.
	scanCap   bool
	scanSeq   []scanEntry    // bounded ordered trace of accesses (the protocol)
	scanCount map[string]int // "REGION+off R|W" -> count (summary histogram)
}

type scanEntry struct {
	cyc       uint64
	pc, val   uint32
	region    string
	off, size uint32
	write     bool
}

// scanWatched classifies a (possibly P0/P2-aliased) address as one of the key-scan
// register blocks, returning the block name and offset within it.
func scanWatched(va uint32) (string, uint32, bool) {
	b := (va & 0x1FFFFFFF) | 0xA0000000
	switch b &^ 0xFFF {
	case 0xA4080000:
		return "INTC", b & 0xFFF, true
	case 0xA44B0000:
		return "KEYSC", b & 0xFFF, true
	case 0xA4050000:
		return "PFC", b & 0xFFF, true
	case 0xA44C0000: // port strobe/clock pins the matrix scan toggles (found cont.18k)
		return "PORTL", b & 0xFFF, true
	case 0xA44D0000:
		return "ETMU2", b & 0xFFF, true
	}
	return "", 0, false
}

// captureScan records one watched access (caller checks b.scanCap).
func (b *MMIOBus) captureScan(va, size, val uint32, write bool) {
	region, off, ok := scanWatched(va)
	if !ok {
		return
	}
	rw := "R"
	if write {
		rw = "W"
	}
	b.scanCount[fmt.Sprintf("%s+%03x %s", region, off, rw)]++
	if len(b.scanSeq) < 6000 {
		var pc uint32
		var cyc uint64
		if b.cpu != nil {
			pc, cyc = b.cpu.pc, b.cpu.cycles
		}
		b.scanSeq = append(b.scanSeq, scanEntry{cyc: cyc, pc: pc, val: val, region: region, off: off, size: size, write: write})
	}
}

func NewMMIOBus() *MMIOBus {
	b := &MMIOBus{unknown: map[uint32]int{}}
	b.periphIRQ = &periphIRQ{base: newBase("PERIPH_IRQ", 0xA4610000, 0x1000), bus: b}
	b.etmu2 = &etmuCounter{base: newBase("ETMU2", 0xA44D0000, 0x1000)}
	b.etmu2.bus = b
	b.keysc = newKeysc()
	b.rtc = newRTC(b)
	b.instrPerSec = 70_000_000
	b.dmac = &dmac{base: newBase("DMAC", 0xFE008000, 0x1000), bus: b}
	b.tmu = newTMU(b)
	b.intc = newINTC()
	b.keysc.bus = b
	pfc := &base{nm: "PFC", bs: 0xA4050000, sz: 0x1000, regs: map[uint32]uint32{}}
	b.lcd = newLCD(pfc)
	b.tx = &txPin{pfc: pfc}
	b.ccn = &ccn{base: newBase("CCN", 0xFF000000, 0x1000)}
	b.cpuopm = &base{nm: "CPUOPM", bs: 0xFF2F0000, sz: 4, regs: map[uint32]uint32{}}
	cpg := &cpg{base: newBase("CPG", 0xA4150000, 0x1000)}
	b.ubc = newUBC(cpg)
	b.regions = []region{
		b.cpuopm,
		cpg,
		b.tx, // before the PFC it is part of
		pfc,
		&base{nm: "WDT", bs: 0xA4520000, sz: 0x1000, regs: map[uint32]uint32{}},
		b.intc,
		b.tmu,
		&etmu{base: newBase("ETMU", 0xA44A0000, 0x1000)},
		b.etmu2,
		b.periphIRQ,
		b.rtc, // listed before FRC: same page, must win the lookup
		&freeCounter{base: newBase("FRC", 0xA4130000, 0x10000)},
		&intx{base: newBase("INTX", 0xA4140000, 0x1000)},
		b.keysc,
		&bcdALU{base: newBase("BCDALU", 0xA4CB0000, 0x1000)},
		&base{nm: "BSC", bs: 0xFEC10000, sz: 0x1000, regs: map[uint32]uint32{}},
		b.dmac,
		b.ccn,
		&utlbArrays{base: newBase("UTLB", 0xF6000000, 0x02000000), c: b.ccn},
		b.lcd,
		b.ubc,
	}
	return b
}

func (b *MMIOBus) find(va uint32) region {
	cands := [3]uint32{va, (va & 0x1FFFFFFF) | 0xA0000000, va & 0x1FFFFFFF}
	for _, c := range cands {
		for _, r := range b.regions {
			if r.contains(c) {
				return r
			}
		}
	}
	return nil
}

// findHit returns the region and the candidate address that matched.
func (b *MMIOBus) findHit(va uint32) (region, uint32) {
	cands := [3]uint32{va, (va & 0x1FFFFFFF) | 0xA0000000, va & 0x1FFFFFFF}
	for _, c := range cands {
		for _, r := range b.regions {
			if r.contains(c) {
				return r, c
			}
		}
	}
	return nil, va
}

func (b *MMIOBus) Read(va, size uint32) uint32 {
	if b.watchBase != 0 && (va&^0xFFF) == b.watchBase && b.cpu != nil {
		b.watchPC[b.cpu.pc]++
	}
	if b.kbEnd != 0 && (va&^0xFFF) == 0xA4080000 && b.cpu != nil &&
		b.cpu.cycles >= b.kbStart && b.cpu.cycles < b.kbEnd {
		if b.kbReg < 0 || uint32(b.kbReg) == (va-0xA4080000)&0xFFFF {
			return b.kbVal
		}
	}
	r, hit := b.findHit(va)
	if r == nil {
		b.unknown[va]++
		return 0
	}
	res := r.read(hit, size)
	if b.scanCap {
		b.captureScan(va, size, res, false)
	}
	return res
}

func (b *MMIOBus) Write(va, size, val uint32) {
	b.dirty = true
	if b.scanCap {
		b.captureScan(va, size, val, true)
	}
	r, hit := b.findHit(va)
	if r == nil {
		b.unknown[va]++
		if b.wcount != nil {
			b.wcount[fmt.Sprintf("unmapped %08x", va)]++
		}
		return
	}
	if b.wcount != nil {
		b.wcount[r.name()]++
	}
	r.write(hit, size, val)
}

// SetInstrPerSecond tells the bus how many instructions the host executes per real second,
// so instruction-based time maps to wall-clock: the KEYSC scan (33 Hz measured), the 32.768 kHz
// free counter, and the RTC (calendar + periodic interrupt) all derive from it.
func (b *MMIOBus) SetInstrPerSecond(ips uint64) {
	if ips < 1_000_000 {
		ips = 1_000_000
	}
	b.instrPerSec = ips
	b.keysc.scanPeriod = ips / KeyScanHz
	b.etmu2.countDiv = ips / 32768
	if b.rtc.rcr2&0x70 != 0 {
		b.rtc.alignPeriodic()
	}
}

// regionRegs returns each region's last-written register map by name (for save-states).
// KEYSC keeps its live state in fields, so it is exported as offsets 0x0C/0x10/0x14.
func (b *MMIOBus) regionRegs() map[string]map[uint32]uint32 {
	out := map[string]map[uint32]uint32{}
	for _, r := range b.regions {
		switch x := r.(type) {
		case *keyscUnit:
			out[x.nm] = map[uint32]uint32{0x0C: x.ctrl, 0x10: x.mode, 0x14: x.ie << 8}
		case *base:
			out[x.nm] = x.regs
		case *cpg:
			out[x.nm] = x.regs
		case *dmac:
			out[x.nm] = x.regs
		case *etmu:
			out[x.nm] = x.regs
		case *etmuCounter:
			m := maps.Clone(x.regs)
			saveTimerChans(m, x.ch[:], x.cycles(), func(int) uint64 { return x.div() })
			out[x.nm] = m
		case *tmu:
			m := maps.Clone(x.regs)
			saveTimerChans(m, x.ch[:], x.cycles(), x.div)
			out[x.nm] = m
		case *intcUnit:
			out[x.nm] = x.stateMap()
		case *ccn:
			out[x.nm] = x.stateMap()
		case *ubcUnit:
			out[x.nm] = maps.Clone(x.regs)
		case *rtc:
			out[x.nm] = map[uint32]uint32{0x1C: x.rcr1, 0x1E: x.rcr2}
		case *lcd:
			out[x.nm] = x.stateMap()
		case *periphIRQ:
			m := make(map[uint32]uint32, len(x.regs)+3)
			for k, v := range x.regs {
				m[k] = v
			}
			if x.adcMode {
				m[adcKeyMode] = 1
			}
			if x.armed && b.cpu != nil {
				m[adcKeyArmed], m[adcKeyLeft] = 1, uint32(x.doneAt-min(x.doneAt, b.cpu.idle))
			}
			if x.swArmed && b.cpu != nil {
				m[adcKeySw], m[adcKeySwLeft] = 1, uint32(x.swDoneAt-min(x.swDoneAt, b.cpu.cycles))
			}
			out[x.nm] = m
		}
	}
	return out
}

// restoreRegionRegs applies a saved register map (see regionRegs). Unknown names are ignored.
func (b *MMIOBus) restoreRegionRegs(saved map[string]map[uint32]uint32) {
	for _, r := range b.regions {
		m, ok := saved[r.name()]
		if !ok {
			continue
		}
		if k, isK := r.(*keyscUnit); isK {
			k.ctrl, k.mode, k.ie, k.flags = m[0x0C], m[0x10], m[0x14]>>8, 0
			k.scanLeft, k.wasHeld = 0, false
			continue
		}
		if x, isR := r.(*rtc); isR {
			x.rcr1, x.rcr2, x.nextPeriodic = m[0x1C], m[0x1E]&0x7F, 0
			continue
		}
		if x, isL := r.(*lcd); isL {
			x.restoreState(m)
			continue
		}
		if x, isC := r.(*ccn); isC {
			x.restoreState(m)
			continue
		}
		if x, isU := r.(*ubcUnit); isU {
			x.restore(m)
			continue
		}
		if x, isI := r.(*intcUnit); isI {
			x.restoreState(m)
			continue
		}
		if x, isP := r.(*periphIRQ); isP {
			for k := range x.regs {
				delete(x.regs, k)
			}
			for k, v := range m {
				if k < adcKeyMode {
					x.regs[k] = v
				}
			}
			x.adcMode, x.armed, x.swArmed = m[adcKeyMode] == 1, m[adcKeyArmed] == 1, m[adcKeySw] == 1
			if b.cpu != nil {
				x.doneAt = b.cpu.idle + uint64(m[adcKeyLeft])
				x.swDoneAt = b.cpu.cycles + uint64(m[adcKeySwLeft])
			}
			continue
		}
		var dst map[uint32]uint32
		switch x := r.(type) {
		case *base:
			dst = x.regs
		case *cpg:
			dst = x.regs
		case *dmac:
			dst = x.regs
		case *etmu:
			dst = x.regs
		case *etmuCounter:
			dst = x.regs
			m = restoreTimerChans(m, x.ch[:], x.cycles(), func(int) uint64 { return x.div() })
		case *tmu:
			dst = x.regs
			// the prescaler (TCR) picks a channel's clock: restore the channels, then their timing
			m = restoreTimerChans(m, x.ch[:], x.cycles(), func(int) uint64 { return 1 })
			for i := range x.ch {
				if c := &x.ch[i]; c.running {
					c.nextUnf = c.startCyc + (uint64(c.tcnt)+1)*x.div(i)
				}
			}
		}
		if dst == nil {
			continue
		}
		for off := range dst {
			delete(dst, off)
		}
		for off, v := range m {
			dst[off] = v
		}
	}
}

// applyLegacyResumeDefaults sets the peripheral state a pre-MMIO-section snapshot needs to
// behave like the machine it was taken on: the display-enable bit in PFC port data
// 0xA405013C bit4 (both LCD push routines return early without it, so the OS would never
// push VRAM to the LCD again) and the KEYSC post-init configuration.
func (b *MMIOBus) applyLegacyResumeDefaults() {
	for _, r := range b.regions {
		if x, ok := r.(*base); ok && x.nm == "PFC" {
			x.regs[0x13C] |= 0x10
		}
	}
	b.keysc.resumeDefaults()
	b.intc.seedOSDefaults()
	b.lcd.restoreState(nil)
}

// FrameSAR returns the source address of any DMAC channel currently programmed to
// push to the LCD area-5 (DAR==0x14000000), i.e. the live framebuffer, if any.
func (b *MMIOBus) FrameSAR() (uint32, bool) {
	for _, r := range b.regions {
		if d, ok := r.(*dmac); ok {
			for _, choff := range []uint32{0, 0x10, 0x20, 0x30} {
				if d.regs[choff+4] == 0x14000000 {
					return d.regs[choff], true
				}
			}
		}
	}
	return 0, false
}

// tick: cycle-driven timer. Every timerPeriod instructions set the PERIPH_IRQ flag
// and request INTEVT 0x560. Gated by cpu.BL/IMASK in accept, so safe to free-run.
func (b *MMIOBus) tick(cpu *CPU) {
	b.keysc.tick(cpu)
	if b.timerPeriod == 0 {
		return // pure-boot mode (goldens): no interrupt sources at all
	}
	b.rtc.tick(cpu)
	b.etmu2.tick(cpu)
	b.tmu.tick(cpu)
	if b.periphIRQ.adcMode {
		b.periphIRQ.adcTick(cpu)
	} else if cpu.cycles >= b.timerNext {
		b.timerNext = cpu.cycles + b.timerPeriod
		b.timerTicks++
		b.periphIRQ.setTimerFlag()
		b.raise(cpu, TimerINTEVT, TimerLevel)
	}
}
