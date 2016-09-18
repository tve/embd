package embd

import "sync"

type spiBusFactory func(int, byte, byte, int, int, int, func() error) SPIBus

type spiDriver struct {
	spiDevMinor int
	initializer func() error

	busMap     map[byte]SPIBus
	busMapLock sync.Mutex

	sbf spiBusFactory
}

// NewSPIDriver returns a SPIDriver interface which allows control
// over the SPI bus.
// It is called by the host specific code (e.g. in host/rpi, host/bbb,
// etc in order to register the host's driver. User code does not generally need to call this.
func NewSPIDriver(spiDevMinor int, sbf spiBusFactory, i func() error) SPIDriver {
	return &spiDriver{
		spiDevMinor: spiDevMinor,
		sbf:         sbf,
		initializer: i,
	}
}

// Bus returns a SPIBus interface which allows us to use spi functionalities
func (s *spiDriver) Bus(mode, channel byte, speed, bpw, delay int) SPIBus {
	s.busMapLock.Lock()
	defer s.busMapLock.Unlock()

	b := s.sbf(s.spiDevMinor, mode, channel, speed, bpw, delay, s.initializer)
	s.busMap = make(map[byte]SPIBus)
	s.busMap[channel] = b
	return b
}

// Close cleans up all the initialized SPIbus
func (s *spiDriver) Close() error {
	for _, b := range s.busMap {
		b.Close()
	}

	return nil
}
