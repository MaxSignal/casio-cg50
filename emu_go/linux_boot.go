package main

// linux_boot.go — bare-metal runner for the fxcg20-linux bundle.
//
// It turns the emulator into an fx-CG10/20 without the Casio OS: 2 MiB of DRAM at
// 0x08000000, the SH7305 identification registers and a CG20-like CPG setup, then loads a
// Linux zImage exactly where the loader add-in leaves it and starts it in the state the
// loader's trampoline hands over (privileged, BL=1, IMASK=15, MMU off).
//
// The console is read back from the R61524 GRAM by matching every 6x8 text cell against the
// kernel's font, so tests can assert on what is on the screen. Keys are typed through the
// KEYSC matrix with the same layout the kernel's keyboard driver implements.
//
//	go run . linux -kernel zImage -font font6x8.h [-ins 400M] [-type "ls /\n"] [-png out.png]

import (
	"bufio"
	"flag"
	"fmt"
	"image"
	"image/color"
	"image/png"
	"os"
	"regexp"
	"strconv"
	"strings"
)

const (
	cg20DramBase = 0x08000000
	cg20DramSize = 0x00200000
	textCols     = 64
	textRows     = 27
)

type glyph [8]uint8

func loadFont(path string) (map[glyph]byte, error) {
	src, err := os.ReadFile(path)
	if err != nil {
		return nil, err
	}
	re := regexp.MustCompile(`\{ ((?:0x[0-9a-f]{2},? ?){8}) \}`)
	m := map[glyph]byte{}
	ch := byte(0x20)
	for _, row := range re.FindAllStringSubmatch(string(src), -1) {
		var g glyph
		for i, f := range strings.Fields(strings.ReplaceAll(row[1], ",", " ")) {
			v, _ := strconv.ParseUint(f, 0, 8)
			g[i] = uint8(v)
		}
		if _, dup := m[g]; !dup {
			m[g] = ch
		}
		ch++
	}
	return m, nil
}

// screenText decodes the panel GRAM into text rows.
func screenText(l *lcd, font map[glyph]byte) []string {
	px := func(x, y int) uint16 { return l.gram[y*panelW+(osOriginH-x)] }
	rows := make([]string, textRows)
	for r := 0; r < textRows; r++ {
		var sb strings.Builder
		for c := 0; c < textCols; c++ {
			counts := map[uint16]int{}
			for y := 0; y < 8; y++ {
				for x := 0; x < 6; x++ {
					counts[px(c*6+x, r*8+y)]++
				}
			}
			bg, best := uint16(0), -1
			for col, n := range counts {
				if n > best || (n == best && col < bg) {
					bg, best = col, n
				}
			}
			var g glyph
			for y := 0; y < 8; y++ {
				for x := 0; x < 6; x++ {
					if px(c*6+x, r*8+y) != bg {
						g[y] |= 0x80 >> x
					}
				}
			}
			ch, ok := font[g]
			if !ok { // the cursor fills the two bottom pixel rows
				g[6], g[7] = g[6]^0xfc, g[7]^0xfc
				ch, ok = font[g]
			}
			if !ok {
				ch = '?'
			}
			sb.WriteByte(ch)
		}
		rows[r] = strings.TrimRight(sb.String(), " ")
	}
	return rows
}

func writePNG(l *lcd, path string) error {
	img := image.NewRGBA(image.Rect(0, 0, FbWidth, FbHeight))
	for y := 0; y < FbHeight; y++ {
		for x := 0; x < FbWidth; x++ {
			p := l.gram[y*panelW+(osOriginH-x)]
			img.Set(x, y, color.RGBA{uint8(p>>11) << 3, uint8(p>>5&0x3f) << 2, uint8(p&0x1f) << 3, 255})
		}
	}
	f, err := os.Create(path)
	if err != nil {
		return err
	}
	defer f.Close()
	return png.Encode(f, img)
}

// Keyboard layout of arch/sh/boards/mach-fxcg/console.c, as (row, col) of the KEYSC matrix.
type kpos struct{ row, col uint32 }

var (
	kShift = kpos{6, 8}
	kAlpha = kpos{6, 7}
	kCaps  = kpos{5, 7}
)

var plainKeys = map[byte]kpos{
	'0': {6, 1}, '.': {5, 1}, ' ': {4, 1}, '_': {3, 1}, '\n': {2, 1},
	'1': {6, 2}, '2': {5, 2}, '3': {4, 2}, '+': {3, 2}, '-': {2, 2},
	'4': {6, 3}, '5': {5, 3}, '6': {4, 3}, '*': {3, 3}, '/': {2, 3},
	'7': {6, 4}, '8': {5, 4}, '9': {4, 4}, '\b': {3, 4},
	':': {6, 5}, '@': {5, 5}, '(': {4, 5}, ')': {3, 5}, ',': {2, 5}, '\\': {1, 5},
	'=': {6, 6}, '\'': {5, 6}, '"': {4, 6}, '#': {3, 6}, '!': {2, 6}, '?': {1, 6},
	'^': {4, 7}, 0x1b: {3, 7}, '\t': {5, 8}, '~': {4, 8},
	'|': {6, 9}, '>': {5, 9}, '<': {4, 9}, '&': {3, 9}, ';': {2, 9}, '$': {1, 9},
	0x03: {0, 0},
}

var shiftKeys = map[byte]kpos{
	'[': {3, 2}, ']': {2, 2}, '{': {3, 3}, '}': {2, 3}, '%': {3, 1}, '`': {4, 7},
}

var alphaKeys = map[byte]kpos{
	'z': {6, 1}, 'u': {6, 2}, 'v': {5, 2}, 'w': {4, 2}, 'x': {3, 2}, 'y': {2, 2},
	'p': {6, 3}, 'q': {5, 3}, 'r': {4, 3}, 's': {3, 3}, 't': {2, 3},
	'm': {6, 4}, 'n': {5, 4}, 'o': {4, 4},
	'g': {6, 5}, 'h': {5, 5}, 'i': {4, 5}, 'j': {3, 5}, 'k': {2, 5}, 'l': {1, 5},
	'a': {6, 6}, 'b': {5, 6}, 'c': {4, 6}, 'd': {3, 6}, 'e': {2, 6}, 'f': {1, 6},
}

// keysFor returns the key presses that type ch.
func keysFor(ch byte) ([]kpos, error) {
	if k, ok := plainKeys[ch]; ok {
		return []kpos{k}, nil
	}
	if k, ok := shiftKeys[ch]; ok {
		return []kpos{kShift, k}, nil
	}
	if k, ok := alphaKeys[ch]; ok {
		return []kpos{kAlpha, k}, nil
	}
	if ch >= 'A' && ch <= 'Z' {
		return []kpos{kAlpha, kShift, alphaKeys[ch+'a'-'A']}, nil
	}
	return nil, fmt.Errorf("no key types %q", ch)
}

func runLinux(args []string) {
	fs := flag.NewFlagSet("linux", flag.ExitOnError)
	kernel := fs.String("kernel", "", "zImage (raw binary)")
	fontPath := fs.String("font", "", "font6x8.h of the kernel console (for text decoding)")
	load := fs.Uint64("load", 0x88040000, "address the loader copies the zImage to")
	maxIns := fs.Uint64("ins", 600_000_000, "instructions to run")
	ips := fs.Uint64("ips", 58_000_000, "emulated instructions per second (sets the time base)")
	typing := fs.String("type", "", "text to type once the shell prompt appears (\\n = EXE)")
	prompt := fs.String("prompt", "# ", "screen text that means the shell is ready for -type")
	pngOut := fs.String("png", "", "write the final screen to this PNG")
	verbose := fs.Bool("v", false, "print the screen whenever it changes")
	logBuf := fs.Uint64("logbuf", 0, "address of the kernel __log_buf to dump at the end")
	logSize := fs.Uint64("logsize", 4096, "size of __log_buf")
	dumps := fs.String("dump", "", "comma separated addr:len memory ranges to hex dump at the end")
	g3a := fs.String("g3a", "", "boot through this loader add-in instead of loading -kernel directly")
	fsFile := fs.String("file", "", "with -g3a: host file served as \\\\fls0\\linux.bin")
	flsFiles := fs.String("fls", "", "with -g3a: more files for the add-in, NAME=PATH,... (\\\\fls0\\NAME)")
	getKeys := fs.String("getkey", "", "with -g3a: the key codes GetKey returns, comma separated (then EXE)")
	flashPath := fs.String("flash", "", "NOR flash image (e.g. a dump of a real calculator); default all 0xFF")
	start := fs.Uint64("start", 0, "without -kernel or -g3a: start the CPU here (a kernel that runs in place from -flash)")
	flashWrites := fs.Bool("flash-writes", false, "report every write to the flash (address, value, pc) and count them")
	flashOut := fs.String("flash-out", "", "write the 32 MiB NOR flash to this file at the end (what the kernel programmed and erased)")
	fs.Parse(args)

	DramBase, DramSize = cg20DramBase, cg20DramSize

	var img []byte
	var err error
	if *g3a == "" && *start == 0 {
		if img, err = os.ReadFile(*kernel); err != nil {
			fmt.Fprintln(os.Stderr, err)
			os.Exit(2)
		}
	}
	font, err := loadFont(*fontPath)
	if err != nil || len(font) < 90 {
		fmt.Fprintln(os.Stderr, "font:", err, len(font))
		os.Exit(2)
	}

	var flash []byte
	if *flashPath != "" {
		if flash, err = os.ReadFile(*flashPath); err != nil {
			fmt.Fprintln(os.Stderr, err)
			os.Exit(2)
		}
	}
	e := NewEmulator(flash)
	m := e.mmio
	m.timerPeriod = 1 << 62 // no Casio OS tick
	m.timerNext = 1 << 62
	e.SetInstrPerSecond(*ips)
	m.ccn.regs[0x30] = 0x10300B00 // PVR: SH-4A family, 0x300B
	m.ccn.regs[0x44] = 0x00002C00 // PRR: SH7305
	m.ccn.regs[0x40] = 0x00880000 // CVR: 32 KiB I/D caches, DSP, no FPU
	for _, r := range m.regions {
		if c, ok := r.(*cpg); ok {
			c.regs[0x00] = 0x1F200303 // FRQCRA: PLL x32, I/8, B/16, P/16 (58.98 MHz CPU)
			c.regs[0x24] = 0x00005000 // PLLCR: PLL and FLL on
			c.regs[0x50] = 0x00004384 // FLLFRQ: SELXM=1, FLF=900
		}
	}
	m.keysc.resumeDefaults()

	cpu := e.cpu
	cpu.sysExceptions = true
	if *flashWrites {
		e.mem.fwrites = map[uint32]int{}
		e.mem.fwLog = 100
	}
	if *g3a != "" {
		files := map[string]string{}
		for _, f := range strings.Split(*flsFiles, ",") {
			if name, path, ok := strings.Cut(f, "="); ok {
				files[name] = path
			}
		}
		var keys []uint32
		for _, k := range strings.Split(*getKeys, ",") {
			var v uint32
			if _, err := fmt.Sscan(k, &v); err == nil {
				keys = append(keys, v)
			}
		}
		if err := startAddin(e, *g3a, *fsFile, files, keys); err != nil {
			fmt.Fprintln(os.Stderr, err)
			os.Exit(2)
		}
	} else if *start != 0 {
		cpu.setSR(srMD | srRB | srBL | srIMASK)
		cpu.vbr = 0
		cpu.pc = uint32(*start)
	} else {
		if uint32(len(img)) > DramSize {
			fmt.Fprintln(os.Stderr, "zImage too large")
			os.Exit(2)
		}
		copy(e.mem.dram[uint32(*load)&0x1FFFFFFF-DramBase:], img)
		cpu.setSR(srMD | srRB | srBL | srIMASK)
		cpu.vbr = 0
		cpu.pc = uint32(*load)
	}

	var keys []kpos
	typed := *typing == ""
	var last string
	const chunk = 2_000_000
	holdUntil, gapUntil := uint64(0), uint64(0)
	var held *kpos
	for cpu.cycles < *maxIns {
		e.Step(chunk)
		if f := e.Fault(); f != "" {
			fmt.Println("FAULT:", f)
			dumpRegs(cpu)
			break
		}
		rows := screenText(m.lcd, font)
		cur := strings.Join(rows, "\n")
		if *verbose && cur != last {
			fmt.Printf("---- %d M instructions ----\n%s\n", cpu.cycles/1_000_000, cur)
		}
		last = cur

		if !typed && strings.HasSuffix(lastNonEmpty(rows[:textRows-1]), strings.TrimSpace(*prompt)) {
			typed = true
			s := strings.NewReplacer(`\n`, "\n", `\t`, "\t", `\e`, "\x1b").Replace(*typing)
			for i := 0; i < len(s); i++ {
				k, err := keysFor(s[i])
				if err != nil {
					fmt.Fprintln(os.Stderr, err)
					os.Exit(2)
				}
				keys = append(keys, k...)
			}
		}
		// one key per few polls of the kernel driver (20 ms each)
		switch {
		case held != nil && cpu.cycles >= holdUntil:
			m.keysc.release(held.row, held.col)
			held = nil
			gapUntil = cpu.cycles + *ips/25
		case held == nil && len(keys) > 0 && cpu.cycles >= gapUntil:
			k := keys[0]
			keys = keys[1:]
			m.keysc.press(k.row, k.col)
			held = &k
			holdUntil = cpu.cycles + *ips/25
		}
	}
	fmt.Printf("==== screen after %d M instructions (pc=%08x) ====\n", cpu.cycles/1_000_000, cpu.pc)
	fmt.Println(last)
	for _, d := range strings.Split(*dumps, ",") {
		if d == "" {
			continue
		}
		var a, n uint32
		fmt.Sscanf(strings.Replace(d, ":", " ", 1), "%v %v", &a, &n)
		for i := uint32(0); i < n; i += 4 {
			if i%16 == 0 {
				fmt.Printf("\n%08x:", a+i)
			}
			fmt.Printf(" %08x", e.mem.R32(a+i))
		}
		fmt.Println()
	}
	if *logBuf != 0 {
		fmt.Println("==== __log_buf ====")
		var sb strings.Builder
		for i := uint64(0); i < *logSize; i++ {
			b := byte(e.mem.R8(uint32(*logBuf + i)))
			if b >= 0x20 && b < 0x7f || b == '\n' {
				sb.WriteByte(b)
			} else if b == 0 {
				continue
			} else {
				sb.WriteByte('\n')
			}
		}
		fmt.Println(sb.String())
	}
	if *pngOut != "" {
		if err := writePNG(m.lcd, *pngOut); err != nil {
			fmt.Fprintln(os.Stderr, err)
		}
	}
	if *flashWrites {
		n := 0
		for _, c := range e.mem.fwrites {
			n += c
		}
		fmt.Printf("==== flash writes: %d ====\n", n)
	}
	if *flashOut != "" {
		if err := os.WriteFile(*flashOut, e.mem.flash, 0o644); err != nil {
			fmt.Fprintln(os.Stderr, err)
		}
	}
	if len(keys) > 0 || !typed {
		fmt.Println("NOTE: not all keys were typed")
	}
}

func lastNonEmpty(rows []string) string {
	for i := len(rows) - 1; i >= 0; i-- {
		if rows[i] != "" {
			return rows[i]
		}
	}
	return ""
}

func dumpRegs(c *CPU) {
	w := bufio.NewWriter(os.Stdout)
	defer w.Flush()
	for i := 0; i < 16; i++ {
		fmt.Fprintf(w, "r%-2d=%08x ", i, c.r[i])
		if i%4 == 3 {
			fmt.Fprintln(w)
		}
	}
	fmt.Fprintf(w, "pc=%08x pr=%08x sr=%08x vbr=%08x spc=%08x ssr=%08x\n", c.pc, c.pr, c.sr, c.vbr, c.spc, c.ssr)
}
