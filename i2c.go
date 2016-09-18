// I2C support.

package embd

// I2CBus interface is used to interact with the I2C bus. The bus is typically opened using
// NewI2CBus.
type I2CBus interface {
	// ReadByte reads a byte from the given address.
	ReadByte(addr byte) (value byte, err error)
	// ReadBytes reads a slice of bytes from the given address.
	ReadBytes(addr byte, num int) (value []byte, err error)
	// WriteByte writes a byte to the given address.
	WriteByte(addr, value byte) error
	// WriteBytes writes a slice bytes to the given address.
	WriteBytes(addr byte, value []byte) error

	// ReadFromReg reads `len(value)` bytes from the given address and register.
	// It performs one I2C write cycle with the register number and then `len(value)`
	// read cycles.
	ReadFromReg(addr, reg byte, value []byte) error
	// ReadByteFromReg reads a byte from the given address and register.
	// It performs one I2C write cycle with the register number and then one read cycle.
	ReadByteFromReg(addr, reg byte) (value byte, err error)
	// ReadU16FromReg reads a unsigned 16 bit integer from the given address and register.
	// It performs one I2C write cycle with the register number and then two read cycles.
	// The uint16 result value is assembled in big-endian order from the two bytes read.
	ReadWordFromReg(addr, reg byte) (value uint16, err error)

	// WriteToReg writes `len(value)` bytes to the given address and register.
	// It performs `len(value)`+1 write cycles with the register number followed by the values.
	WriteToReg(addr, reg byte, value []byte) error
	// WriteByteToReg writes a byte to the given address and register.
	// It performs two write cycles with the register number followed by the value.
	WriteByteToReg(addr, reg, value byte) error
	// WriteU16ToReg writes a 16-bit integer to the given address and register.
	// It performs three write cycles with the register number followed by the value
	// in big-endian order.
	WriteWordToReg(addr, reg byte, value uint16) error

	// Close releases the resources associated with the bus.
	Close() error
}

// I2CDriver interface interacts with the host descriptors to allow us
// control of I2C communication.
type I2CDriver interface {
	// Bus returns an I2CBus
	Bus(l byte) I2CBus

	// Close releases the resources associated with the driver.
	Close() error
}

var i2cDriverInitialized bool
var i2cDriverInstance I2CDriver

// InitI2C initializes the I2C driver after detecting the current host. It is not usually called,
// rather NewI2CBus should be called, which calls InitI2C.
func InitI2C() error {
	if i2cDriverInitialized {
		return nil
	}

	desc, err := DescribeHost()
	if err != nil {
		return err
	}

	if desc.I2CDriver == nil {
		return ErrFeatureNotSupported
	}

	i2cDriverInstance = desc.I2CDriver()
	i2cDriverInitialized = true

	return nil
}

// CloseI2C releases resources associated with the I2C driver.
func CloseI2C() error {
	return i2cDriverInstance.Close()
}

// NewI2CBus initializes the I2C driver appropriate for the currect host (calling InitI2C) and
// returns an I2CBus. The parameter `l` is an index into the available I2C devices. The generic
// driver uses `/dev/i2c-`l but other drivers may interpret the parameter differently.
func NewI2CBus(l byte) I2CBus {
	if err := InitI2C(); err != nil {
		panic(err)
	}

	return i2cDriverInstance.Bus(l)
}
