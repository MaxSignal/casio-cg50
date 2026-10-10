package main

// addin_hle.go — run a .g3a add-in the way the fx-CG10/20 Casio OS would, without the OS.
//
// The add-in's code is mapped at 0x00300000 (1 MiB UTLB page onto a copy in "flash") and its
// 512 KiB RAM at 0x08100000 (64 KiB pages onto physical 0x08160000, the CG20 add-in RAM
// area); the stack starts at 0x881E0000 and the OS heap is 0x881E0000..0x881FFFFF. The few
// syscalls the fxcg-linux loader uses are emulated at the syscall dispatcher 0x80020070.
// This is enough to test the loader's staging, UTLB translation and hand-over trampoline.

import (
	"fmt"
	"os"
	"strings"
)

const (
	addinCode     = 0x00300000
	addinRAM      = 0x08100000
	addinRAMPhys  = 0x08160000
	addinStackTop = 0x881E0000
	heapStart     = 0x881E0000
	heapEnd       = 0x88200000
	syscallEntry  = 0x80020070
)

// startAddin runs the add-in g3aPath. files are what \\fls0\NAME opens (host
// paths by name; filePath is linux.bin), keys the codes GetKey returns in turn
// (EXE once they are used up).
func startAddin(e *Emulator, g3aPath, filePath string, files map[string]string, keys []uint32) error {
	g, err := os.ReadFile(g3aPath)
	if err != nil {
		return err
	}
	if len(g) < 0x7004 {
		return fmt.Errorf("%s: not a g3a", g3aPath)
	}
	code := g[0x7000 : len(g)-4]
	copy(e.mem.flash[addinCode:], code)

	contents := map[string][]byte{}
	if filePath != "" {
		files[`linux.bin`] = filePath
	}
	for name, path := range files {
		data, err := os.ReadFile(path)
		if err != nil {
			return err
		}
		contents[`\\fls0\`+name] = data
	}
	open := map[uint32][]byte{}

	c := e.mmio.ccn
	const flags = 0x100 | 0x60 | 0x8 | 0x4 | 0x2 // V, PR=RW, C, D, SH
	c.utlb[0] = tlbEntry{vpn: addinCode, data: addinCode | flags | 0x90} // 1 MiB
	for i := uint32(0); i < 8; i++ {
		c.utlb[1+i] = tlbEntry{vpn: addinRAM + i<<16, data: (addinRAMPhys + i<<16) | flags | 0x80} // 64 KiB
	}
	c.write(0xFF000010, 4, 1) // MMUCR.AT
	c.regs[ccnMMUCR] = 1

	cpu := e.cpu
	cpu.setSR(srMD) // privileged, bank 0, interrupts open like an add-in
	cpu.vbr = 0x80000000
	cpu.r[15] = addinStackTop
	cpu.pr = 0x80000000 // returning from the add-in would land in "the OS"
	cpu.pc = addinCode

	heap := uint32(heapStart + 0x100)
	nextFd := uint32(3)
	str := func(a uint32) string {
		var sb strings.Builder
		for i := uint32(0); i < 64; i++ {
			b := byte(e.mem.R8(a + i))
			if b == 0 {
				break
			}
			sb.WriteByte(b)
		}
		return sb.String()
	}
	cpu.hlePC = syscallEntry
	cpu.hle = func() bool {
		r := &cpu.r
		ret := uint32(0)
		switch r[0] {
		case 0x0029, 0x0272, 0x025F, 0x0921, 0x02B7: // exec check, Bdisp_*, color, status area
		case 0x18F9: // PrintXY(x, y, "xx"+text, mode, color)
			fmt.Printf("[addin] %2d: %s\n", r[5], strings.TrimPrefix(str(r[6])[2:], ""))
		case 0x0EAB: // GetKey(&key): the next of keys, then EXE
			k := uint32(30004)
			if len(keys) > 0 {
				k, keys = keys[0], keys[1:]
			}
			e.mem.W32(r[4], k)
			ret = 1
		case 0x01E6: // GetVRAMAddress
			ret = 0xA8000000
		case 0x1DA3: // Bfile_OpenFile_OS(u16 *name, mode, 0)
			var sb strings.Builder
			for a := r[4]; ; a += 2 {
				ch := e.mem.R16(a)
				if ch == 0 {
					break
				}
				sb.WriteByte(byte(ch))
			}
			name := sb.String()
			fmt.Printf("[addin] open %q\n", name)
			if data, ok := contents[name]; ok {
				open[nextFd] = data
				ret = nextFd
				nextFd++
			} else {
				ret = 0xFFFFFFFF
			}
		case 0x1DA6: // Bfile_GetFileSize_OS
			ret = uint32(len(open[r[4]]))
		case 0x1DAC: // Bfile_ReadFile_OS(fd, buf, size, pos)
			buf, n, pos := r[5], r[6], r[7]
			file, ok := open[r[4]]
			if !ok || pos > uint32(len(file)) {
				ret = 0xFFFFFFFF
				break
			}
			if pos+n > uint32(len(file)) {
				n = uint32(len(file)) - pos
			}
			for i := uint32(0); i < n; i++ {
				e.mem.W8(buf+i, uint32(file[pos+i]))
			}
			ret = n
		case 0x1DA4: // Bfile_CloseFile_OS
			delete(open, r[4])
		case 0x1F44: // sys_malloc(size)
			n := (r[4] + 7) &^ 7
			if heap+n+8 <= heapEnd {
				ret = heap
				heap += n + 8 // a little header, like a real allocator
			}
		default:
			panic(fmt.Sprintf("unemulated syscall 0x%04x", r[0]))
		}
		r[0] = ret
		cpu.pc = cpu.pr
		return true
	}
	return nil
}
