### am32 note
- PB4 use as TMR3 input capture (DMA1-CH4) (see define in HW_GROUP_B4)
- if use Dshot, B4 may switch dynamic as IO (also dma cfg)
- see DMA1_CH4 irq: HalfDT and FullDT. (pwm has half DT?)
- D1CH4 fullDT trigger EXTI 15 evt => EXTI15 use to process DSHOT data
- do nothing in TIM3 (incap) except clear int flag
- ic_timer_prescaler=119, fcpu=120mhz. each tim cnt=1us
- PB4 IC TIM3CH1 cfg: 0x41 = 0100: f𝑆𝐴𝑀𝑃𝐿𝐼𝑁𝐺=f𝐷𝑇𝑆/2, N=6 0001 = 00 nodivide 01: Input, C1IN is mapped on C1IFP1
- DMA: no HalfDT. 98a or 98b = FullDT AM32 的定时器捕获 DMA 缓冲区存的正是每个上升沿的计数值,并不是直接是脉宽。


### vars explain?
- inputSet 1=input ok
- armed 0=stop, 1=ready
- input
- 连续32个脉宽最小值需要>200 <20000

