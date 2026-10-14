package main

// boot20: run an fx-CG10/fx-CG20 flash image from reset, as the calculator does when it
// is switched on (boot code, then the OS), and report where the CPU spent its time: the
// boot code (the first 128 KiB of the flash), the OS image after it, or RAM. Used to see,
// without reading any code, whether the boot code goes on to the OS or stops before it
// (for example at its "OS update" screen) for a given flash image.
//
//	cgemu boot20 -flash IMAGE [-ins N] [-png SCREEN] [-every N] [-exe N] [-writes]
//
// -exe presses a key every N instructions, the -keys in turn (to go through the first-boot
// screens), -writes
// counts the OS's writes to the flash by 128 KiB sector.

import (
	"flag"
	"fmt"
	"os"
	"sort"
	"strings"
)

func runBoot20(args []string) {
	fs := flag.NewFlagSet("boot20", flag.ExitOnError)
	flashPath := fs.String("flash", "", "32 MiB flash image")
	maxIns := fs.Uint64("ins", 400_000_000, "instructions to run")
	every := fs.Uint64("every", 20_000, "sample the PC every N instructions")
	pngOut := fs.String("png", "", "write the final screen to this PNG")
	exeEvery := fs.Uint64("exe", 0, "press a key every N instructions (0: never)")
	keyList := fs.String("keys", "2,1", "with -exe: the keys pressed in turn, round and round (row,col ...; 2,1 is EXE)")
	writes := fs.Bool("writes", false, "count the writes to the flash by 128 KiB sector")
	flashOut := fs.String("flash-out", "", "write the flash to this file at the end")
	shots := fs.Uint64("shots", 0, "with -png: also save the screen every N instructions (PNG-N.png)")
	fs.Parse(args)

	flash, err := os.ReadFile(*flashPath)
	if err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(2)
	}
	DramBase, DramSize = cg20DramBase, cg20DramSize
	e := NewEmulator(flash)
	cpu := e.cpu
	if *writes {
		e.mem.fwrites = map[uint32]int{}
	}
	nextExe := *exeEvery
	var keys [][2]uint32
	for _, k := range strings.Fields(*keyList) {
		var r, c uint32
		fmt.Sscanf(k, "%d,%d", &r, &c)
		keys = append(keys, [2]uint32{r, c})
	}
	nkey := 0
	nextShot, nshot := *shots, 0

	// where the PC was, by 64 KiB block of the physical address space
	samples := map[uint32]uint64{}
	firstOS := uint64(0)
	for cpu.cycles < *maxIns {
		e.Step(int(*every))
		if *exeEvery != 0 && cpu.cycles >= nextExe {
			k := keys[nkey%len(keys)]
			e.InjectKey(k[0], k[1])
			nkey++
			nextExe += *exeEvery
		}
		if f := e.Fault(); f != "" {
			fmt.Println("FAULT:", f)
			dumpRegs(cpu)
			break
		}
		if *shots != 0 && *pngOut != "" && cpu.cycles >= nextShot {
			writePNG(e.mmio.lcd, fmt.Sprintf("%s-%03d.png", strings.TrimSuffix(*pngOut, ".png"), nshot))
			nshot++
			nextShot += *shots
		}
		phys := cpu.pc & 0x1FFFFFFF
		samples[phys&^0xFFFF]++
		if firstOS == 0 && phys >= 0x20000 && phys < 0x00C00000 {
			firstOS = cpu.cycles
		}
	}
	var boot, osimg, ram, other uint64
	for b, n := range samples {
		switch {
		case b < 0x20000:
			boot += n
		case b < 0x00C00000:
			osimg += n
		case b >= DramBase&0x1FFFFFFF && b < (DramBase+DramSize)&0x1FFFFFFF:
			ram += n
		default:
			other += n
		}
	}
	fmt.Printf("ran %d M instructions, pc=%08x\n", cpu.cycles/1_000_000, cpu.pc)
	fmt.Printf("samples: boot code %d, OS image %d, RAM %d, other %d\n", boot, osimg, ram, other)
	if firstOS != 0 {
		fmt.Printf("first in the OS image after %d instructions\n", firstOS)
	} else {
		fmt.Println("never in the OS image")
	}
	var blocks []uint32
	for b := range samples {
		blocks = append(blocks, b)
	}
	sort.Slice(blocks, func(i, j int) bool { return samples[blocks[i]] > samples[blocks[j]] })
	for i, b := range blocks {
		if i == 8 {
			break
		}
		fmt.Printf("  %08x-%08x: %d\n", b, b+0xFFFF, samples[b])
	}
	if *writes {
		bySector := map[uint32]int{}
		for page, n := range e.mem.fwrites {
			bySector[page&^0x1FFFF] += n
		}
		var secs []uint32
		for sec := range bySector {
			secs = append(secs, sec)
		}
		sort.Slice(secs, func(i, j int) bool { return secs[i] < secs[j] })
		fmt.Println("flash writes by 128 KiB sector:")
		for _, sec := range secs {
			fmt.Printf("  %08x: %d\n", sec, bySector[sec])
		}
	}
	if *flashOut != "" {
		if err := os.WriteFile(*flashOut, e.mem.flash, 0o644); err != nil {
			fmt.Fprintln(os.Stderr, err)
		}
	}
	if *pngOut != "" {
		if err := writePNG(e.mmio.lcd, *pngOut); err != nil {
			fmt.Fprintln(os.Stderr, err)
		}
	}
}
