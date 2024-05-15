uint64 timestamp                        # time since system start (microseconds)

float32 maximum_fuel_capacity       	# maximum fuel capacity
float32 consumed_fuel       		# consumed fuel, NaN if unavailable
float32 fuel_consumption_rate     	# fuel consumption rate

uint8 percent_remaining                 # percentage of remaining fuel
float32 remaining_fuel      		# remaining fuel

uint8 fuel_tank_id                      # identifier for the fuel tank

uint32 fuel_type                        # type of fuel based on MAV_FUEL_TYPE enum
uint8 MAV_FUEL_TYPE_UNKNOWN = 0		# fuel type not specified. Fuel levels are normalized (i.e., maximum is 1, and other levels are relative to 1).
uint8 MAV_FUEL_TYPE_BATTERY = 1		# represents electrical energy storage in batteries. Fuel levels are measured in Ampere-hours (Ah), and flow rates in Amps (A).
uint8 MAV_FUEL_TYPE_LIQUID = 2		# represents generic liquid fuels, such as gasoline or diesel. Fuel levels are measured in millilitres (ml), and flow rates in millilitres per second (ml/s).

float32 temperature                     # fuel temperature in Kelvin, NaN if not measured
