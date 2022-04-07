config-pin -a p9.17 i2c
config-pin -a p9.18 i2c
config-pin -a p9.19 i2c
config-pin -a p9.20 i2c
config-pin -a p9.11 uart
config-pin -a p9.13 uart
config-pin -a p9.24 uart
config-pin -a p9.26 uart

echo "Configuring I2C Pins"
config-pin -q p9.17
config-pin -q p9.18
config-pin -q p9.19
config-pin -q p9.20
echo ""

echo "Configuring Uart Pins"
config-pin -q p9.11
config-pin -q p9.13
config-pin -q p9.24
config-pin -q p9.26