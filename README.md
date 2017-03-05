### Linux on Technicolor TC7200

```
[    0.000000] Linux version 4.10.0+ (user@host) (gcc version 6.2.1 20161124 (Ubuntu 6.2.1-5ubuntu1) ) #9 SMP Sun Mar 5 12:23:58 CET 2017
[    0.000000] CPU0 revision is: 0002a080 (Broadcom BMIPS4350)
[    0.000000] MIPS: machine is Technicolor TC7200 (Viper)
[    0.000000] Determined physical RAM map:
[    0.000000]  memory: 08000000 @ 00000000 (usable)
[    0.000000] earlycon: bcm63xx_uart0 at MMIO 0x14e00500 (options '')
[    0.000000] bootconsole [bcm63xx_uart0] enabled
[    0.000000] Initrd not found or empty - disabling initrd
[    0.000000] Primary instruction cache 64kB, VIPT, 4-way, linesize 16 bytes.
[    0.000000] Primary data cache 32kB, 2-way, VIPT, cache aliases, linesize 16 bytes
[    0.000000] This processor doesn't support highmem. -131072k highmem ignored
[    0.000000] Zone ranges:
[    0.000000]   Normal   [mem 0x0000000000000000-0x0000000007ffffff]
[    0.000000]   HighMem  empty
[    0.000000] Movable zone start for each node
[    0.000000] Early memory node ranges
[    0.000000]   node   0: [mem 0x0000000000000000-0x0000000007ffffff]
[    0.000000] Initmem setup node 0 [mem 0x0000000000000000-0x0000000007ffffff]
[    0.000000] percpu: Embedded 12 pages/cpu @81107000 s25744 r0 d23408 u49152
[    0.000000] Built 1 zonelists in Zone order, mobility grouping on.  Total pages: 32512
[    0.000000] Kernel command line: console=ttyS0,115200 earlycon
[    0.000000] PID hash table entries: 512 (order: -1, 2048 bytes)
[    0.000000] Dentry cache hash table entries: 16384 (order: 4, 65536 bytes)
[    0.000000] Inode-cache hash table entries: 8192 (order: 3, 32768 bytes)
[    0.000000] Memory: 119672K/131072K available (5732K kernel code, 193K rwdata, 800K rodata, 3124K init, 276K bss, 11400K reserved, 0K cma-reserved, 0K highmem)
[    0.000000] SLUB: HWalign=128, Order=0-3, MinObjects=0, CPUs=1, Nodes=1
[    0.000000] Hierarchical RCU implementation.
[    0.000000]  Build-time adjustment of leaf fanout to 32.
[    0.000000]  RCU restricting CPUs from NR_CPUS=4 to nr_cpu_ids=1.
[    0.000000] RCU: Adjusting geometry for rcu_fanout_leaf=32, nr_cpu_ids=1
[    0.000000] NR_IRQS:128
[    0.000000] irq_bcm7120_l2: registered BCM3380 L2 intc (mem: 0xb4e00048, parent IRQ(s): 1)
[    0.000000] irq_bcm7120_l2: registered BCM3380 L2 intc (mem: 0xb51f8048, parent IRQ(s): 1)
[    0.000000] clocksource: MIPS: mask: 0xffffffff max_cycles: 0xffffffff, max_idle_ns: 6370868154 ns
[    0.000736] sched_clock: 32 bits at 300MHz, resolution 3ns, wraps every 7158278654ns
[    0.011908] Calibrating delay loop... 4.28 BogoMIPS (lpj=8576)
[    0.094780] pid_max: default: 32768 minimum: 301
[    0.112001] Mount-cache hash table entries: 1024 (order: 0, 4096 bytes)
[    0.121399] Mountpoint-cache hash table entries: 1024 (order: 0, 4096 bytes)
[    0.328736] smp: Bringing up secondary CPUs ...
[    0.335778] smp: Brought up 1 node, 1 CPU
[    0.377067] devtmpfs: initialized
[    0.566913] clocksource: jiffies: mask: 0xffffffff max_cycles: 0xffffffff, max_idle_ns: 7645041785100000 ns
[    0.580067] futex hash table entries: 256 (order: 3, 32768 bytes)
[    0.638229] NET: Registered protocol family 16
[    1.294841] random: fast init done
[    4.753451] SCSI subsystem initialized
[    4.790205] usbcore: registered new interface driver usbfs
[    4.803907] usbcore: registered new interface driver hub
[    4.817288] usbcore: registered new device driver usb
[    5.244075] clocksource: Switched to clocksource MIPS
[    5.591565] NET: Registered protocol family 2
[    5.690418] TCP established hash table entries: 1024 (order: 0, 4096 bytes)
[    5.705969] TCP bind hash table entries: 1024 (order: 1, 8192 bytes)
[    5.720554] TCP: Hash tables configured (established 1024 bind 1024)
[    5.738235] UDP hash table entries: 256 (order: 1, 8192 bytes)
[    5.750261] UDP-Lite hash table entries: 256 (order: 1, 8192 bytes)
[    5.777709] NET: Registered protocol family 1
[    5.836884] RPC: Registered named UNIX socket transport module.
[    5.845409] RPC: Registered udp transport module.
[    5.852561] RPC: Registered tcp transport module.
[    5.859772] RPC: Registered tcp NFSv4.1 backchannel transport module.
[   47.660720] workingset: timestamp_bits=30 max_order=15 bucket_order=0
[   49.688219] fuse init (API version 7.26)
[   50.720326] io scheduler noop registered (default)
[   54.574151] random: crng init done
[   69.769487] 14e00500.serial: ttyS0 at MMIO 0x14e00500 (irq = 8, base_baud = 1687500) is a bcm63xx_uart
[   69.783936] console [ttyS0] enabled
[   69.783936] console [ttyS0] enabled
[   69.793663] bootconsole [bcm63xx_uart0] disabled
[   69.793663] bootconsole [bcm63xx_uart0] disabled
[   70.055991] libphy: Fixed MDIO Bus: probed
[   70.076914] usbcore: registered new interface driver asix
[   70.091967] usbcore: registered new interface driver ax88179_178a
[   70.107741] usbcore: registered new interface driver cdc_ether
[   70.122179] usbcore: registered new interface driver net1080
[   70.137496] usbcore: registered new interface driver cdc_subset
[   70.153090] usbcore: registered new interface driver zaurus
[   70.169529] usbcore: registered new interface driver cdc_ncm
[   70.177692] ehci_hcd: USB 2.0 'Enhanced' Host Controller (EHCI) Driver
[   70.187832] ehci-platform: EHCI generic platform driver
[   70.201490] ohci_hcd: USB 1.1 'Open' Host Controller (OHCI) Driver
[   70.211683] ohci-platform: OHCI generic platform driver
[   70.232300] usbcore: registered new interface driver usb-storage
[   70.340053] NET: Registered protocol family 10
[   70.929434] Segment Routing with IPv6
[   70.943325] sit: IPv6, IPv4 and MPLS over IPv4 tunneling driver
[   71.060360] NET: Registered protocol family 17
[   71.819632] Freeing unused kernel memory: 3124K
[   71.827874] This architecture does not have kernel memory protection.
init started: BusyBox v1.11.2 (2012-06-18 18:04:15 CEST)

Please press Enter to activate this console.
```


