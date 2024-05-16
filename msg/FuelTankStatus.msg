uint64 timestamp                        # time since system start (microseconds)

float32 maximum_fuel_capacity       	# maximum fuel capacity. Must always be provided, either from the driver or a parameter
float32 consumed_fuel       		# consumed fuel, NaN if not measured. Should not be inferred from the max fuel capacity
float32 fuel_consumption_rate     	# fuel consumption rate, NaN if not measured

uint8 percent_remaining                 # percentage of remaining fuel, UINT8_MAX if not provided
float32 remaining_fuel      		# remaining fuel, NaN if not measured. Should not be inferred from the max fuel capacity

uint8 fuel_tank_id                      # identifier for the fuel tank. Must match ID of other messages for same fuel system. 0 by default when only a single tank exists

uint32 fuel_type                        # type of fuel based on MAV_FUEL_TYPE enum. Set to MAV_FUEL_TYPE_UNKNOWN if unknown or it does not fit the provided types
uint8 MAV_FUEL_TYPE_UNKNOWN = 0		# fuel type not specified. Fuel levels are normalized (i.e., maximum is 1, and other levels are relative to 1)
uint8 MAV_FUEL_TYPE_BATTERY = 1		# represents electrical energy storage in batteries. Fuel levels are measured in Ampere-hours (Ah), and flow rates in Amps (A)
uint8 MAV_FUEL_TYPE_LIQUID = 2		# represents generic liquid fuels, such as gasoline or diesel. Fuel levels are measured in millilitres (ml), and flow rates in millilitres per second (ml/s)

float32 temperature                     # fuel temperature in Kelvin, NaN if not measured
