    //–– 1) Set up a base pointer and a value
    loadi 0 0x02       // R0 ← 2 (base address)
    loadi 1 0x05       // R1 ← 5  (payload)

    //–– 2) Byte‐store and byte‐load
    swi   1 0x00       // M[0] ← low byte of R1 = 0x05
    lwi   2 0x00       // R2 ← zero_extend(M[0]) = 5

    //–– 3) Add a constant into R2
    loadi 3 0x03       // R3 ← 3
    add   2 2 3        // R2 ← R2 + R3 = 5 + 3 = 8

    //–– 4) Word‐store and word‐load
    swd   2 0          // M[R0] ← R2 = 8
    lwd   3 0          // R3 ← M[R0] = 8

    //–– 5) Subtract the original payload
    sub   3 3 1        // R3 ← R3 – R1 = 8 – 5 = 3
