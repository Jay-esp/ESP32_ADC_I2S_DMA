Reading analog pins is slow, for an audio project i needed to sample two analog ports at high speed.
It is possible to read the ADC at high speed using DMA and I2S however i could not find any example on how to do it on two ports, plenty of examples for one port.
Eventually i found a solution so i wanted to share it.

