// Copyright 2016 by Thorsten von Eicken, see LICENSE file

// The RFM69 package interfaces with a HopeRF RFM69 radio connected to an SPI bus. In addition,
// an interrupt capable GPIO pin may be used to avoid having to poll the radio.
package rfm69

import (
	"fmt"
	"log"
	"time"

	"github.com/kidoman/embd"
)

// rfm69 represents a HopeRF RFM69 radio
type rfm69 struct {
	// configuration
	spi     embd.SPIBus       // bus where the radio is connected
	intrPin embd.InterruptPin // interrupt pin for RX and TX interrupts
	intrCnt int               // count interrupts
	sync    []byte            // sync bytes
	freq    uint32            // center frequency
	rate    uint32            // bit rate from table
	// state
	mode byte // current operation mode
	// info about current RX packet
	rxInfo *RxInfo
	rxChan chan RxPacket
}

// Rate describes the RFM69 configuration to achieve a specific bit rate
type Rate struct {
	Fdev    int  // TX frequency deviation in Hz
	Shaping byte // 0:none, 1:gaussian BT=1, 2:gaussian BT=0.5, 3:gaussian BT=0.3
	RxBw    byte // value for rxBw register (0x19)
	AfcBw   byte // value for afcBw register (0x1A)
}

// Rates is the table of supported bit rates and their corresponding register settings. The map
// key is the bit rate in bits per second.
var Rates = map[uint32]Rate{
	49230: {45000, 0, 0x4A, 0x42}, // used by jeelabs driver
	50000: {45000, 0, 0x4A, 0x42}, // nice round number
}

// TemporaryError is an interface that is implemented by errors that may be temporary, i.e.,
// that are retryable
type TemporaryError interface {
	Temporary() bool // true if the error is temporary (retryable)
}

type temporaryError string

func (te temporaryError) Error() string   { return string(te) }
func (te temporaryError) Temporary() bool { return true }

/*
type Packet struct {
	Length  uint8 // number of message bytes plus 1 for the address byte
	Address uint8 // destination address
	Message []byte
}
*/

// RxPacket is a received packet with stats
type RxPacket struct {
	Payload []byte  // payload, from address to last data byte, excluding length & crc
	CrcOK   bool    // whether received CRC is OK
	Crc     uint16  // received CRC
	Info    *RxInfo // stats about the reception
}

// RxInfo contains stats about a received packet
type RxInfo struct {
	rssi int // rssi value for current packet
	lna  int // low noise amp gain for current packet
	fei  int // frequency error for current packet
	afc  int // frequency correction applied for current packet
}

// New creates a connection to an rfm69 radio connected to the provided SPI bus and interrupt pin.
// the bufCount determines how many transmit buffers are allocated to allow for the queueing of
// transmit packets.
// For the RFM69 the SPI bus must be set to 10Mhz and mode 0.
func New(bus embd.SPIBus, intr embd.InterruptPin, sync []byte, freq, rate uint32) *rfm69 {
	return &rfm69{spi: bus, intrPin: intr, sync: sync, freq: freq, rate: rate, mode: 255}
}

// writeReg writes one or multiple registers starting at addr, the rfm69 auto-increments (except
// for the FIFO register where that wouldn't be desirable)
func (rf *rfm69) writeReg(addr byte, data ...byte) error {
	buf := make([]byte, len(data)+1) // having to allocate is annoying, sigh
	buf[0] = addr | 0x80
	copy(buf[1:], data)
	return rf.spi.TransferAndReceiveData(buf)
}

// readReg reads one register and returns its value
func (rf *rfm69) readReg(addr byte) (byte, error) {
	buf := []byte{addr & 0x7f, 0}
	err := rf.spi.TransferAndReceiveData(buf)
	return buf[1], err
}

// Init initializes the rfm69 radio and places it in receive mode. Received packet will be sent on
// the returned channel, which has a small amount of buffering.
func (rf *rfm69) Init() (<-chan RxPacket, error) {
	// try to establish communication with the rfm69
	sync := func(pattern byte) error {
		for n := 10; n > 0; n-- {
			rf.writeReg(REG_SYNCVALUE1, pattern)
			v, err := rf.readReg(REG_SYNCVALUE1)
			if err != nil {
				return err
			}
			if v == pattern {
				return nil
			}
		}
		return fmt.Errorf("Cannot sync with rfm69 chip")
	}
	if err := sync(0xaa); err != nil {
		return nil, err
	}
	if err := sync(0x55); err != nil {
		return nil, err
	}

	if err := rf.setMode(MODE_SLEEP); err != nil {
		return nil, err
	}
	embd.SetDirection("CSID1", embd.Out)
	embd.DigitalWrite("CSID1", 1)
	embd.DigitalWrite("CSID1", 0)

	vers, err := rf.readReg(REG_VERSION)
	if err == nil {
		log.Printf("RFM69/SX1231 version %#x", vers)
	}

	// write the configuration into the registers
	for i := 0; i < len(configRegs)-1; i += 2 {
		if err := rf.writeReg(configRegs[i], configRegs[i+1]); err != nil {
			return nil, err
		}
	}

	// configure the bit rate and frequency
	if err := rf.SetRate(rf.rate); err != nil {
		return nil, err
	}
	rf.SetFrequency(rf.freq)

	// configure the sync bytes
	if len(rf.sync) < 1 || len(rf.sync) > 8 {
		return nil, fmt.Errorf("invalid number of sync bytes: %d, must be 1..8", len(rf.sync))
	}
	buf := make([]byte, len(rf.sync)+2)
	buf[0] = REG_SYNCCONFIG | 0x80
	buf[1] = byte(0x80 + ((len(rf.sync) - 1) << 3))
	copy(buf[2:], rf.sync)
	if err := rf.spi.TransferAndReceiveData(buf); err != nil {
		return nil, err
	}

	// allocate a channel for recevied packets, give it some buffer but the reality is that
	// packets don't come in that fast either...
	if rf.rxChan == nil {
		rf.rxChan = make(chan RxPacket, 10)
	}

	// initialize interrupts
	// we use a bit of a dirty hack to get hold of the digital pin underlying the
	// interrupt pin so we can set its direction to be an input, although maybe that's
	// taken care of by the linux driver? who knows...
	if rf.intrPin == nil {
		return nil, fmt.Errorf("interrupt pin missing")
	}
	gpio, ok := rf.intrPin.(embd.DigitalPin)
	if ok {
		//log.Printf("Set intr direction")
		gpio.SetDirection(embd.In)
	}
	// somehow the interrupts don't always initialize properly, so we test them and we
	// try again, which usually works in less that 2 iterations, but it's a bit random!
	for {
		if err := rf.intrPin.Watch(embd.EdgeRising, rf.intrHandler); err != nil {
			return nil, err
		}

		// test the interrupts
		rf.setMode(MODE_FS)
		rf.writeReg(REG_DIOMAPPING1, DIO_MAPPING+0xC0)
		time.Sleep(time.Second / 100)
		if rf.intrCnt != 0 {
			break
		}
		log.Printf("oops, interrupt pin malfunction, retrying")
		rf.intrPin.StopWatching()
		rf.intrPin.Watch(embd.EdgeNone, rf.intrHandler)
		rf.intrPin.StopWatching()
		rf.writeReg(REG_DIOMAPPING1, DIO_MAPPING)
	}

	// finally turn on the receiver
	if err := rf.setMode(MODE_RECEIVE); err != nil {
		return nil, err
	}

	// log register contents
	rf.logRegs()
	embd.DigitalWrite("CSID1", 1)
	embd.DigitalWrite("CSID1", 0)
	embd.DigitalWrite("CSID1", 1)
	embd.DigitalWrite("CSID1", 0)

	return rf.rxChan, nil
}

func (rf *rfm69) logRegs() {
	regs := make([]byte, 0x50)
	regs[0] = 1
	err := rf.spi.TransferAndReceiveData(regs)
	if err != nil {
		log.Printf("Can't read all regs: %s", err.Error())
		return
	}
	for i := 0; i < len(regs)-4; i += 4 {
		log.Printf("%#02x: %#02x %#02x %#02x %#02x", i,
			regs[i], regs[i+1], regs[i+2], regs[i+3])
	}
}

// SetFrequency changes the center frequency at which the radio transmits and receives. The
// frequency can be specified at any scale (hz, khz, mhz). The frequency value is not checked
// and invalid values will simply cause the radio not to work particularly well.
func (rf *rfm69) SetFrequency(freq uint32) {
	// accept any frequency scale as input, including KHz and MHz
	// multiply by 10 until freq >= 100 MHz
	for freq > 0 && freq < 100000000 {
		freq = freq * 10
	}

	// Frequency steps are in units of (32,000,000 >> 19) = 61.03515625 Hz
	// use multiples of 64 to avoid multi-precision arithmetic, i.e. 3906.25 Hz
	// due to this, the lower 6 bits of the calculated factor will always be 0
	// this is still 4 ppm, i.e. well below the radio's 32 MHz crystal accuracy
	// 868.0 MHz = 0xD90000, 868.3 MHz = 0xD91300, 915.0 MHz = 0xE4C000
	frf := (freq << 2) / (32000000 >> 11)
	rf.writeReg(REG_FRFMSB, byte(frf>>10), byte(frf>>2), byte(frf<<6))
	//log.Printf("SetFreq: %d %d %02x %02x %02x", freq, uint32(frf<<6),
	//	byte(frf>>10), byte(frf>>2), byte(frf<<6))
}

// SetRate sets the bit rate according to the Rates table. The `rate` parameter must use one of
// the values from the table.
func (rf *rfm69) SetRate(rate uint32) error {
	r, found := Rates[rate]
	if !found {
		return fmt.Errorf("invalid bit rate")
	}

	// program bit rate, assume a 32Mhz osc
	var rateVal uint32 = (32000000 + rate/2) / rate
	rf.writeReg(REG_BITRATEMSB, byte(rateVal>>8), byte(rateVal&0xff))
	// program frequency deviation
	var fStep float64 = 32000000.0 / 524288 // 32Mhz osc / 2^19 = 61.03515625 Hz
	fdevVal := uint32((float64(r.Fdev) + fStep/2) / fStep)
	rf.writeReg(REG_FDEVMSB, byte(fdevVal>>8), byte(fdevVal&0xFF))
	// program data modulation register
	rf.writeReg(REG_DATAMODUL, r.Shaping&0x3)
	// program RX bandwidth and AFC bandwidth
	rf.writeReg(REG_RXBW, r.RxBw, r.AfcBw)
	// set AFC mode
	rf.writeReg(REG_AFCCTRL, 0x20)

	return nil
}

// setMode changes the radio's operating mode and changes the interrupt cause (if necessary), and
// then waits for the new mode to be reached
func (rf *rfm69) setMode(mode byte) error {
	mode = mode & 0x1c

	// if we're in the right mode then don't do anything
	if rf.mode == mode {
		return nil
	}

	// set the interrupt mode if necessary
	switch mode {
	case MODE_TRANSMIT:
		if err := rf.writeReg(REG_DIOMAPPING1, DIO_MAPPING+DIO_PKTSENT); err != nil {
			return err
		}
	case MODE_RECEIVE:
		if err := rf.writeReg(REG_DIOMAPPING1, DIO_MAPPING+DIO_SYNC); err != nil {
			return err
		}
	default:
		// mode used when switching, make sure we don't get an interupt
		if err := rf.writeReg(REG_DIOMAPPING1, DIO_MAPPING); err != nil {
			return err
		}
	}

	// set the new mode
	if err := rf.writeReg(REG_OPMODE, mode); err != nil {
		rf.mode = 255
		return err
	}

	// busy-wait 'til the new mode is reached
	for {
		val, err := rf.readReg(REG_IRQFLAGS1)
		if err != nil {
			rf.mode = 255
			return err
		}
		if val&IRQ1_MODEREADY != 0 {
			rf.mode = mode
			if mode == MODE_RECEIVE {
				rf.writeReg(REG_DIOMAPPING1, DIO_MAPPING+DIO_RSSI)
			}
			return nil
		}
	}
}

// SetPower configures the radio for the specified output power (TODO: should be in dBm)
func (rf *rfm69) SetPower(v byte) {
	if v > 0x1F {
		v = 0x1F
	}
	log.Printf("SetPower %ddBm", -18+int(v))
	rf.writeReg(0x11, 0x80+v)
}

// receiving checks whether a reception is currently in progress. It uses the PLL-lock flag as the
// earliest indication that something is coming in that is not noise.
func (rf *rfm69) receiving() bool {
	// can't be receiving if we're not in the right mode...
	if rf.mode != MODE_RECEIVE {
		return false
	}
	irq1, err := rf.readReg(REG_IRQFLAGS1)
	if err != nil {
		return false
	}
	irq2, err := rf.readReg(REG_IRQFLAGS2)
	//log.Printf("Rcv? %#02x %#02x", irq1, irq2)
	return err == nil && (irq1&IRQ1_SYNADDRMATCH != 0 || irq2&IRQ2_PAYLOADREADY != 0)
}

// Send ensures that we're not in the midst of receiving a packet and then transmits it. If
// we are in the midst of receiving a packet it returns an error that implements the
// TemporaryError interface.
func (rf *rfm69) Send(payload []byte) error {
	embd.DigitalWrite("CSID1", 1)
	embd.DigitalWrite("CSID1", 0)
	if len(payload) > 65 {
		return fmt.Errorf("message too long: %d, max is 65", len(payload))
	}

	if rf.receiving() {
		return temporaryError(fmt.Sprintf("cannot transmit: in the midst of a reception"))
	}

	if err := rf.setMode(MODE_FS); err != nil {
		return err
	}
	//rf.writeReg(0x2D, 0x01) // set preamble to 1 (too short)
	//rf.writeReg(0x2F, 0x00) // set wrong sync value

	// push the message into the FIFO
	buf := make([]byte, len(payload)+1)
	buf[0] = byte(len(payload) + 1)
	copy(buf[1:], payload)
	if err := rf.writeReg(REG_FIFO|0x80, buf...); err != nil {
		return err
	}
	if err := rf.setMode(MODE_TRANSMIT); err != nil {
		return err
	}
	/*
		for i := 30; i > 0; i-- {
			val, err := rf.readReg(REG_IRQFLAGS2)
			if err != nil {
				return err
			}
			m, _ := rf.readReg(REG_OPMODE)
			v2, _ := rf.readReg(REG_IRQFLAGS1)
			log.Printf("mode: %#02x flags: %#02x %#02x", m, v2, val)
			if val&IRQ2_PACKETSENT != 0 {
				return nil
			}
		}
		log.Printf("tx timeout")
	*/
	return nil
}

// readInfo reads various reception stats about the arriving/arrived packet and populates
// an RxInfo struct with it.
func (rf *rfm69) readInfo() *RxInfo {
	// collect rxinfo, start with rssi
	rxInfo := &RxInfo{}
	//cfg, _ := rf.readReg(REG_RSSICONFIG)
	//if cfg&0x02 == 0 {
	//rxInfo.rssi = 0 // RSSI not ready
	//} else {
	rssi, _ := rf.readReg(REG_RSSIVALUE)
	rxInfo.rssi = 0 - int(rssi)/2
	//}
	// low noise amp gain
	lna, _ := rf.readReg(REG_LNAVALUE)
	rxInfo.lna = int((lna >> 3) & 0x7)
	// auto freq correction applied, caution: signed value
	buf := []byte{REG_AFCMSB, 0, 0}
	rf.spi.TransferAndReceiveData(buf)
	f := int(int8(buf[1]))<<8 | int(buf[2])
	rxInfo.afc = (f * (32000000 >> 13)) >> 6
	// freq error detected, caution: signed value
	buf = []byte{REG_FEIMSB, 0, 0}
	rf.spi.TransferAndReceiveData(buf)
	f = int(int8(buf[1]))<<8 | int(buf[2])
	rxInfo.fei = (f * (32000000 >> 13)) >> 6
	//fmt.Printf("\nrxinfo: %+v", *rxInfo)
	return rxInfo
}

// intrHandler is the main interrupt handler entry point
func (rf *rfm69) intrHandler(pin embd.DigitalPin) {
	embd.DigitalWrite("CSID1", 1)
	defer func() { embd.DigitalWrite("CSID1", 0) }()
	rf.intrCnt++

	// what this interrupt is about depends on the current mode
	switch rf.mode {
	case MODE_RECEIVE:
		rf.intrReceive()
	case MODE_TRANSMIT:
		rf.intrTransmit()
	default:
		log.Printf("Spurious interrupt in mode=%x", rf.mode)
	}
}

// intrTransmit handles an interrupt while transmitting
func (rf *rfm69) intrTransmit() {
	log.Printf("TX interrupt")
	// double-check that the packet got transmitted
	irq2, err := rf.readReg(REG_IRQFLAGS2)
	if err == nil && irq2&IRQ2_PACKETSENT != 0 {
		// awesome, packet sent, now receive
		rf.setMode(MODE_FS)
		rf.setMode(MODE_RECEIVE)
	} // else hope for another interrupt?
}

func (rf *rfm69) intrReceive() {
	// quickly capture the RX stats
	i0 := rf.readInfo()
	dbgPush("RX Interrupt")
	dbgPush(fmt.Sprintf("%+v", *i0))

	defer dbgPrint()

	/*defer func() {
		rf.setMode(MODE_STANDBY)
		rf.setMode(MODE_RECEIVE)
	}()*/

	// assume we get an interrupt when rssi exceeds threshold, now just loop and
	// collect data...
	t0 := time.Now()
	for {
		// see whether we have a full packet
		irq2, err := rf.readReg(REG_IRQFLAGS2)
		if err != nil || irq2&IRQ2_PAYLOADREADY != 0 {
			break
		}
		irq1, _ := rf.readReg(REG_IRQFLAGS1)
		if irq1&IRQ1_MODEREADY == 0 {
			return
		}
		// timeout so we don't get stuck here
		if time.Since(t0).Seconds() > 100.0*8/50000 { // 100 bytes @50khz
			dbgPush("   timeout")
			return
		}
	}

	i := rf.readInfo()
	dbgPush(fmt.Sprintf("%+v", *i))
	// got packet, read it by fetching the entire FIFO, should be faster than first
	// looking at the length
	buf := make([]byte, 67)
	buf[0] = REG_FIFO
	if err := rf.spi.TransferAndReceiveData(buf); err != nil {
		return
	}
	// push packet into channel
	l := buf[1]
	if l > 66 {
		l = 66 // or error?
	}
	pkt := RxPacket{Payload: buf[2 : 1+l], Info: i}
	select {
	case rf.rxChan <- pkt: // awesome
	default:
		log.Printf("RFM69: RxChan full")
	}
}
