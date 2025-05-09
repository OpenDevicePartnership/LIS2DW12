# ST LIS2DW12 Accelerometer

A `#[no_std]` platform-agnostic driver for the
[LIS2DW12](https://www.st.com/resource/en/datasheet/lis2dw12.pdf)
accelerometer using the [embedded-hal](https://docs.rs/embedded-hal) traits.

## I2C Address Specification
The LIS2DW12 can take one of 2 I2C addresses depending on the connection to the SA0 pin.
| SA0 | Addr |
|-----|------|
| GND | 0x18 |
| V+  | 0x19 |

The driver constructors require an explicit declaration of the SA0 pin state.

## Usage

```rust,ignore
use lis2dw12::{self, Lis2dw12, Register, registers};

// Initialize the driver with the SA0 configuration
let mut accel = Lis2dw12::new(i2c, SA0::Gnd);
let mut accel = Lis2dw12::new(i2c, SA0::Vplus);
let mut accel = Lis2dw12::new_with_sa0_gnd(i2c);
let mut accel = Lis2dw12::new_with_sa0_vplus(i2c);

// Set a desired configuration for each of the 7 control registers
accel.write_reg(Register::Control1, registers::ControlReg1::new(
   lis2dw12::Control1LowPowerMode::LowPower2,
   lis2dw12::Control1ModeSelect::OnDemand,
   lis2dw12::Control1DataRate::HiLo200Hz).into()).await?;

// Read 3D accel data
match accel.acc().await {
   Ok((accx, accy, accz)) => {
      defmt::info!("Accel data: X={}, Y={}, Z={}", accx, accy, accz);
   },
   Err(e) => {
      defmt::error!("Unable to read accel data! {:?}", e);
   }
}

```