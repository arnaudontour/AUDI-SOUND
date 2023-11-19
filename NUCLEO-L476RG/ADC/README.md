# ADC with DMA

## Configuration

- Resolution: 12bit (0 ~ 4096)
- SYSCLK: 80MHz
- Clock prescaler: 4
- Oversampling ratio: 64x (0 ~ (4096*64)/2^6=4096)
- Oversampling right shift: 6bit

## DMA interrupt cycle

```
1/80000000*4*(247.5+12.5)*1024*64 = every 0.85sec
```
## UART output

Vref

```
-
3978 @ 235995
-
3978 @ 236847
-
3978 @ 237699
-
3978 @ 238551
```

GND

```
-
0 @ 269222
-
0 @ 270074
-
0 @ 270926
-
0 @ 271778
```
