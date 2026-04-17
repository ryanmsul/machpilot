import React, { useEffect, useMemo, useRef, useState } from 'react';
import GaugeComponent from 'react-gauge-component';
import * as ROSLIB from 'roslib';
import ros from '../rosConnection';
import useROSSubscription from '../useROSSubscription';

const MAX_TEMPERATURE_KELVIN = 2000;
const TEMPERATURE_BAR_MAX_KELVIN = 2500;
const TEMP_FIELD_HINTS = ['temp', 'temperature', 'egt', 'therm'];
const THROAT_FIELD_CANDIDATES = [
  'throat_area',
  'throatArea',
  'nozzle_throat_area',
  'nozzleThroatArea',
];
const PRESSURE_SENSOR_1_CANDIDATES = [
  'pressure_1',
  'pressure1',
  'pressure_sensor_1',
  'pressureSensor1',
  'sensor_1_pressure',
  'sensor1Pressure',
  'p1',
  'engine_box_pressure',
];
const PRESSURE_SENSOR_2_CANDIDATES = [
  'pressure_2',
  'pressure2',
  'pressure_sensor_2',
  'pressureSensor2',
  'sensor_2_pressure',
  'sensor2Pressure',
  'p2',
];

const GENERIC_TEMP_SENSORS = Array.from({ length: 12 }, (_, index) => ({
  label: `Temp ${index + 1}`,
  candidates: [
    `temp_${index + 1}`,
    `temp${index + 1}`,
    `temperature_${index + 1}`,
    `temperature${index + 1}`,
    `sensor_${index + 1}_temp`,
    `sensor${index + 1}Temp`,
    `thermocouple_${index + 1}`,
    `thermocouple${index + 1}`,
    `tc_${index + 1}`,
    `tc${index + 1}`,
    `t_${index + 1}`,
    `t${index + 1}`,
  ],
}));

const getNumericValue = (source, candidates) => {
  if (!source) {
    return null;
  }

  for (const candidate of candidates) {
    const value = source[candidate];
    if (typeof value === 'number' && Number.isFinite(value)) {
      return value;
    }
  }

  return null;
};

const getAnyNumericValue = (sources, candidates) => {
  for (const source of sources) {
    const value = getNumericValue(source, candidates);
    if (value !== null) {
      return value;
    }
  }

  return null;
};

const discoverTemperatureFields = (sources) => {
  const discovered = [];

  sources.forEach((source) => {
    if (!source) {
      return;
    }

    Object.entries(source).forEach(([key, value]) => {
      const normalizedKey = key.toLowerCase();
      const isTemperatureField = TEMP_FIELD_HINTS.some((hint) => normalizedKey.includes(hint));

      if (
        isTemperatureField &&
        typeof value === 'number' &&
        Number.isFinite(value) &&
        !discovered.some((field) => field.key === key)
      ) {
        discovered.push({
          key,
          value,
        });
      }
    });
  });

  return discovered;
};

const formatNumber = (value, digits = 0) => {
  if (typeof value !== 'number' || !Number.isFinite(value)) {
    return '--';
  }

  return value.toFixed(digits);
};

const formatTemperature = (value) => {
  if (typeof value !== 'number' || !Number.isFinite(value)) {
    return '--';
  }

  return `${value.toFixed(0)} C`;
};

const formatPressure = (value) => {
  if (typeof value !== 'number' || !Number.isFinite(value)) {
    return '--';
  }

  if (value >= 1000) {
    const scaledValue = value / 1000;
    return Number.isInteger(scaledValue) ? `${scaledValue.toFixed(0)} bar` : `${scaledValue.toFixed(1)} bar`;
  }

  return `${value.toFixed(0)} mbar`;
};

const celsiusToKelvin = (value) => value + 273.15;
const kelvinToCelsius = (value) => value - 273.15;

const EngineDashboard = () => {
  const engineData = useROSSubscription('/h20pro/engine_data', 'interfaces/msg/EngineData');
  const pumpRpm = useROSSubscription('/h20pro/pump_rpm', 'interfaces/msg/engine_data2');
  const throttle = useROSSubscription('/h20pro/throttle_command', 'std_msgs/msg/Float32');
  const fuelAmbient = useROSSubscription('/h20pro/fuel_ambient', 'interfaces/msg/FuelAmbient');
  const glowPlugs = useROSSubscription('/h20pro/glow_plugs', 'interfaces/msg/GlowPlugs');
  const lastRunInfo = useROSSubscription('/h20pro/last_run_info', 'interfaces/msg/LastRunInfo');
  const ngReg = useROSSubscription('/h20pro/ng_reg', 'interfaces/msg/NgReg');
  const statistics = useROSSubscription('/h20pro/statistics', 'interfaces/msg/Statistics');
  const systemInfo = useROSSubscription('/h20pro/system_info', 'interfaces/msg/SystemInfo');
  const systemInfo2 = useROSSubscription('/h20pro/system_info2', 'interfaces/msg/PumpRpm');
  const voltageCurrent = useROSSubscription('/h20pro/voltage_current', 'interfaces/msg/VoltageCurrent');

  const telemetrySources = useMemo(
    () => [engineData, fuelAmbient, glowPlugs, lastRunInfo, ngReg, statistics, systemInfo, systemInfo2, voltageCurrent, pumpRpm],
    [engineData, fuelAmbient, glowPlugs, lastRunInfo, ngReg, statistics, systemInfo, systemInfo2, voltageCurrent, pumpRpm]
  );

  const throttlePublisher = new ROSLIB.Topic({
    ros,
    name: '/h20pro/throttle_command',
    messageType: 'std_msgs/msg/Float32',
  });

  const primeAction = new ROSLIB.Action({
    ros,
    name: '/h20pro/prime',
    actionType: 'interfaces/action/Prime',
    timeout: 5,
  });

  const startAction = new ROSLIB.Action({
    ros,
    name: '/h20pro/start',
    actionType: 'interfaces/action/Start',
    timeout: 5,
  });

  const starterTestAction = new ROSLIB.Action({
    ros,
    name: '/h20pro/starter_test',
    actionType: 'interfaces/action/StarterTest',
    timeout: 5,
  });

  const killService = useMemo(
    () =>
      new ROSLIB.Service({
        ros,
        name: '/h20pro/kill',
        serviceType: 'std_srvs/srv/Trigger',
      }),
    []
  );

  const stateName = engineData ? engineData.state_name : 'UNKNOWN';
  const fuelFlow = fuelAmbient ? fuelAmbient.fuel_flow : 0;
  const fuelConsumed = fuelAmbient ? fuelAmbient.fuel_consumed : 0;
  const ambientTemperature = fuelAmbient ? fuelAmbient.ambient_temperature : null;
  const engineBoxPressure = fuelAmbient ? fuelAmbient.engine_box_pressure : 0;
  const throttleCommand = throttle ? throttle.data : 0;
  const throatArea = getAnyNumericValue(telemetrySources, THROAT_FIELD_CANDIDATES);
  const pressureSensor1 = getAnyNumericValue(telemetrySources, PRESSURE_SENSOR_1_CANDIDATES) ?? engineBoxPressure;
  const pressureSensor2 = getAnyNumericValue(telemetrySources, PRESSURE_SENSOR_2_CANDIDATES);

  const discoveredTemperatureFields = useMemo(
    () => discoverTemperatureFields(telemetrySources),
    [telemetrySources]
  );

  const temperatureSensors = useMemo(() => {
    const assignedKeys = new Set();
    const sensorValues = [
      {
        label: 'Temp 1',
        value: engineData && typeof engineData.egt === 'number' ? engineData.egt : null,
      },
      {
        label: 'Temp 2',
        value: ambientTemperature,
      },
    ];

    discoveredTemperatureFields.forEach((field) => {
      if (field.key === 'egt' || field.key === 'ambient_temperature') {
        assignedKeys.add(field.key);
      }
    });

    GENERIC_TEMP_SENSORS.slice(2).forEach((sensor) => {
      const directValue = getAnyNumericValue(telemetrySources, sensor.candidates);
      const directKey = sensor.candidates.find((candidate) =>
        telemetrySources.some((source) => source && typeof source[candidate] === 'number' && Number.isFinite(source[candidate]))
      );

      if (directKey) {
        assignedKeys.add(directKey);
      }

      const fallbackField = discoveredTemperatureFields.find((field) => !assignedKeys.has(field.key));

      if (fallbackField) {
        assignedKeys.add(fallbackField.key);
      }

      sensorValues.push({
        label: sensor.label,
        value: directValue ?? fallbackField?.value ?? null,
      });
    });

    return sensorValues.slice(0, 12);
  }, [ambientTemperature, discoveredTemperatureFields, engineData, telemetrySources]);

  const [isRunningStartTest, setIsRunningStartTest] = useState(false);
  const [startTestButtonText, setStartTestButtonText] = useState('Start Test');
  const [isPriming, setIsPriming] = useState(false);
  const [primeButtonText, setPrimeButtonText] = useState('Prime');
  const [isStarting, setIsStarting] = useState(false);
  const [startButtonText, setStartButtonText] = useState('Start');
  const [isKilling, setIsKilling] = useState(false);
  const [killButtonText, setKillButtonText] = useState('Kill Engine');
  const [throttleValue, setThrottleValue] = useState(0);
  const [useSimulatedTemperatures, setUseSimulatedTemperatures] = useState(false);
  const [simulatedTemperatureCelsius, setSimulatedTemperatureCelsius] = useState(kelvinToCelsius(300));
  const hasTriggeredOvertempKill = useRef(false);

  const displayedTemperatureSensors = useMemo(() => {
    if (!useSimulatedTemperatures) {
      return temperatureSensors;
    }

    return temperatureSensors.map((sensor) => ({
      ...sensor,
      value: simulatedTemperatureCelsius,
    }));
  }, [simulatedTemperatureCelsius, temperatureSensors, useSimulatedTemperatures]);

  const startTestButton = () => {
    setIsRunningStartTest(true);
    setStartTestButtonText('Running Test...');

    starterTestAction.sendGoal(
      {},
      () => {
        setIsRunningStartTest(false);
        setStartTestButtonText('Start Test');
      },
      (feedback) => {
        console.log(feedback);
      }
    );
  };

  const primeButton = () => {
    setIsPriming(true);
    setPrimeButtonText('Priming...');

    primeAction.sendGoal(
      { pump_power_percent: 10 },
      () => {
        setIsPriming(false);
        setPrimeButtonText('Prime');
      },
      (feedback) => {
        console.log(feedback);
      }
    );
  };

  const startButton = () => {
    setIsStarting(true);
    setStartButtonText('Starting...');

    startAction.sendGoal(
      {},
      () => {
        setIsStarting(false);
        setStartButtonText('Start');
      },
      (feedback) => {
        console.log(feedback);
      }
    );
  };

  const killEngine = () => {
    setIsKilling(true);
    setKillButtonText('Killing...');

    killService.callService({}, () => {
      setIsKilling(false);
      setKillButtonText('Kill Engine');
    });
  };

  useEffect(() => {
    const hottestTemperatureCelsius = displayedTemperatureSensors.reduce((maxValue, sensor) => {
      if (typeof sensor.value !== 'number' || !Number.isFinite(sensor.value)) {
        return maxValue;
      }

      return Math.max(maxValue, sensor.value);
    }, -Infinity);

    const hottestTemperatureKelvin =
      hottestTemperatureCelsius === -Infinity ? null : celsiusToKelvin(hottestTemperatureCelsius);

    if (hottestTemperatureKelvin !== null && hottestTemperatureKelvin >= MAX_TEMPERATURE_KELVIN) {
      if (!hasTriggeredOvertempKill.current && !isKilling) {
        hasTriggeredOvertempKill.current = true;
        setIsKilling(true);
        setKillButtonText(`Auto Kill @ ${MAX_TEMPERATURE_KELVIN} K`);

        killService.callService({}, () => {
          setIsKilling(false);
          setKillButtonText('Kill Engine');
        });
      }

      return;
    }

    hasTriggeredOvertempKill.current = false;
  }, [displayedTemperatureSensors, isKilling, killService]);

  const handleThrottleChange = (event) => {
    const newValue = parseFloat(event.target.value);
    setThrottleValue(newValue);
    throttlePublisher.publish({ data: newValue });
  };

  const handleSimulatedTemperatureChange = (event) => {
    setSimulatedTemperatureCelsius(parseFloat(event.target.value));
  };

  const simulatedTemperatureKelvin = celsiusToKelvin(simulatedTemperatureCelsius);

  const stateHeaderStyle = {
    ...cardTitleStyle,
    fontSize: '42px',
    textAlign: 'left',
    color: stateName === 'RUNNING' ? '#73ff9d' : '#ffffff',
  };

  return (
    <div className="dashboard engine-dashboard" style={dashboardStyle}>
      <div style={dashboardContentStyle}>
        <div style={columnStyle}>
          <div style={controlPanelStyle}>
            <div style={controlPanelHeaderStyle}>Engine Controls</div>
            <button onClick={primeButton} disabled={isPriming} style={primaryButtonStyle}>
              {primeButtonText}
            </button>
            <button onClick={startButton} disabled={isStarting} style={primaryButtonStyle}>
              {startButtonText}
            </button>
            <button onClick={killEngine} disabled={isKilling} style={dangerButtonStyle}>
              {killButtonText}
            </button>
            <button onClick={startTestButton} disabled={isRunningStartTest} style={secondaryButtonStyle}>
              {startTestButtonText}
            </button>
            <div style={sliderCardStyle}>
              <div style={metricCardLabelStyle}>Throttle Command</div>
              <div style={metricCardValueStyle}>{formatNumber(throttleCommand, 0)}%</div>
              <input
                type="range"
                min="0"
                max="100"
                step="1"
                value={throttleValue}
                onChange={handleThrottleChange}
                style={sliderStyle}
              />
            </div>
            <div style={sliderCardStyle}>
              <div style={metricCardLabelStyle}>Temperature Simulation</div>
              <div style={smallInfoValueStyle}>{useSimulatedTemperatures ? 'Enabled' : 'Disabled'}</div>
              <button
                onClick={() => setUseSimulatedTemperatures((currentValue) => !currentValue)}
                style={useSimulatedTemperatures ? dangerButtonStyle : secondaryButtonStyle}
              >
                {useSimulatedTemperatures ? 'Disable Temp Sim' : 'Enable Temp Sim'}
              </button>
              <div style={metricCardLabelStyle}>Injected Sensor Value</div>
              <div style={metricCardValueStyle}>
                {formatNumber(simulatedTemperatureCelsius, 0)} C / {formatNumber(simulatedTemperatureKelvin, 0)} K
              </div>
              <input
                type="range"
                min="0"
                max="2000"
                step="10"
                value={simulatedTemperatureCelsius}
                onChange={handleSimulatedTemperatureChange}
                style={sliderStyle}
              />
              <div style={simulationHintStyle}>
                Auto-kill trips at {MAX_TEMPERATURE_KELVIN} K ({formatNumber(kelvinToCelsius(MAX_TEMPERATURE_KELVIN), 0)} C).
              </div>
            </div>
          </div>
        </div>

        <div style={centerColumnStyle}>
          <div style={headerCardStyle}>
            <div style={stateHeaderStyle}>State: {stateName}</div>
            <div style={stateMetaGridStyle}>
              <div style={smallInfoCardStyle}>
                <div style={smallInfoLabelStyle}>Fuel Consumed</div>
                <div style={smallInfoValueStyle}>{formatNumber(fuelConsumed, 0)} mL</div>
              </div>
              <div style={smallInfoCardStyle}>
                <div style={smallInfoLabelStyle}>Engine Box Pressure</div>
                <div style={smallInfoValueStyle}>{formatPressure(engineBoxPressure)}</div>
              </div>
              <div style={smallInfoCardStyle}>
                <div style={smallInfoLabelStyle}>Throat Area</div>
                <div style={smallInfoValueStyle}>
                  {throatArea !== null ? `${formatNumber(throatArea, 2)} mm^2` : '--'}
                </div>
              </div>
            </div>
          </div>

          <div style={topTelemetryGridStyle}>
            <div style={panelCardStyle}>
              <div style={cardTitleStyle}>Pressure Sensor 1</div>
              <div style={mediumGaugeStyle}>
                <GaugeComponent
                  arc={{
                    startAngle: 180,
                    endAngle: 0,
                    subArcs: [
                      { limit: 1000, color: '#5BE12C', showTick: true },
                      { limit: 2500, color: '#F5CD19', showTick: true },
                      { limit: 5000, color: '#EA4228', showTick: true },
                    ],
                    width: 0.25,
                    padding: 0.003,
                  }}
                  labels={{
                    valueLabel: {
                      style: mediumValueLabelStyle,
                      formatTextValue: formatPressure,
                    },
                    tickLabels: {
                      type: 'outer',
                      defaultTickValueConfig: {
                        formatTextValue: (value) => value.toFixed(0),
                        style: mediumTickLabelStyle,
                      },
                      ticks: [{ value: 1000 }, { value: 2000 }, { value: 3000 }, { value: 4000 }, { value: 5000 }],
                    },
                  }}
                  value={pressureSensor1 ?? 0}
                  maxValue={5000}
                />
              </div>
            </div>

            <div style={panelCardStyle}>
              <div style={cardTitleStyle}>Pressure Sensor 2</div>
              <div style={mediumGaugeStyle}>
                <GaugeComponent
                  arc={{
                    startAngle: 180,
                    endAngle: 0,
                    subArcs: [
                      { limit: 1000, color: '#5BE12C', showTick: true },
                      { limit: 2500, color: '#F5CD19', showTick: true },
                      { limit: 5000, color: '#EA4228', showTick: true },
                    ],
                    width: 0.25,
                    padding: 0.003,
                  }}
                  labels={{
                    valueLabel: {
                      style: mediumValueLabelStyle,
                      formatTextValue: formatPressure,
                    },
                    tickLabels: {
                      type: 'outer',
                      defaultTickValueConfig: {
                        formatTextValue: (value) => value.toFixed(0),
                        style: mediumTickLabelStyle,
                      },
                      ticks: [{ value: 1000 }, { value: 2000 }, { value: 3000 }, { value: 4000 }, { value: 5000 }],
                    },
                  }}
                  value={pressureSensor2 ?? 0}
                  maxValue={5000}
                />
              </div>
            </div>

            <div style={sideTelemetryStackStyle}>
              <div style={panelCardStyle}>
                <div style={cardTitleStyle}>Fuel Flow Rate</div>
                <div style={mediumGaugeStyle}>
                  <GaugeComponent
                    arc={{
                      startAngle: 180,
                      endAngle: 0,
                      subArcs: [
                        { limit: 150, color: '#5BE12C', showTick: true },
                        { limit: 300, color: '#F5CD19', showTick: true },
                        { limit: 600, color: '#EA4228', showTick: true },
                      ],
                      width: 0.25,
                      padding: 0.003,
                    }}
                    labels={{
                      valueLabel: {
                        style: mediumValueLabelStyle,
                        formatTextValue: (value) => `${value.toFixed(0)} mL/min`,
                      },
                      tickLabels: {
                        type: 'outer',
                        defaultTickValueConfig: {
                          style: mediumTickLabelStyle,
                          formatTextValue: (value) => value.toFixed(0),
                        },
                        ticks: [{ value: 150 }, { value: 300 }, { value: 450 }, { value: 600 }],
                      },
                    }}
                    value={fuelFlow}
                    maxValue={600}
                  />
                </div>
              </div>
            </div>
          </div>

          <div style={panelCardStyle}>
            <div style={cardTitleStyle}>Temperature Sensors</div>
            <div style={temperatureBarChartStyle}>
              {displayedTemperatureSensors.map((sensor) => (
                <div key={sensor.label} style={temperatureBarColumnStyle}>
                  <div style={temperatureBarValueStyle}>{formatTemperature(sensor.value)}</div>
                  <div style={temperatureBarTrackStyle}>
                    <div
                      style={{
                        ...temperatureBarFillStyle,
                        height: `${Math.max(
                          4,
                          Math.min(
                            ((sensor.value ?? 0) / kelvinToCelsius(TEMPERATURE_BAR_MAX_KELVIN)) * 100,
                            100
                          )
                        )}%`,
                        opacity: sensor.value !== null ? 1 : 0.3,
                      }}
                    />
                  </div>
                  <div style={temperatureBarLabelStyle}>{sensor.label}</div>
                </div>
              ))}
            </div>
            {useSimulatedTemperatures ? (
              <div style={simulationHintStyle}>
                Simulated temperatures are active. All temperature bars are using the injected test value.
              </div>
            ) : null}
          </div>
        </div>
      </div>
    </div>
  );
};

const dashboardStyle = {
  minHeight: '100vh',
  padding: '20px',
  background: 'linear-gradient(180deg, #061018 0%, #0c1d2a 100%)',
  fontFamily: 'Arial, sans-serif',
  color: '#ffffff',
  boxSizing: 'border-box',
};

const dashboardContentStyle = {
  display: 'grid',
  gridTemplateColumns: '280px minmax(0, 1fr)',
  gap: '18px',
  alignItems: 'start',
};

const columnStyle = {
  display: 'flex',
  flexDirection: 'column',
  gap: '16px',
};

const centerColumnStyle = {
  display: 'flex',
  flexDirection: 'column',
  gap: '16px',
};

const panelCardStyle = {
  background: 'rgba(9, 25, 38, 0.92)',
  border: '1px solid rgba(126, 171, 201, 0.25)',
  borderRadius: '18px',
  padding: '18px',
  boxShadow: '0 14px 28px rgba(0, 0, 0, 0.18)',
};

const headerCardStyle = {
  ...panelCardStyle,
  paddingBottom: '14px',
};

const cardTitleStyle = {
  fontSize: '24px',
  fontWeight: 'bold',
  fontFamily: 'MW Font, Arial, sans-serif',
  marginBottom: '14px',
};

const controlPanelStyle = {
  ...panelCardStyle,
  display: 'flex',
  flexDirection: 'column',
  gap: '12px',
  position: 'sticky',
  top: '20px',
};

const controlPanelHeaderStyle = {
  fontSize: '22px',
  fontWeight: 'bold',
  fontFamily: 'MW Font, Arial, sans-serif',
  marginBottom: '4px',
};

const baseButtonStyle = {
  border: 'none',
  borderRadius: '12px',
  padding: '14px 18px',
  fontSize: '18px',
  fontFamily: 'MW Font, Arial, sans-serif',
  fontWeight: 'bold',
  cursor: 'pointer',
  transition: 'transform 0.2s ease',
};

const primaryButtonStyle = {
  ...baseButtonStyle,
  backgroundColor: '#2e8b57',
  color: '#03150d',
};

const secondaryButtonStyle = {
  ...baseButtonStyle,
  backgroundColor: '#d08c2e',
  color: '#1c1203',
};

const dangerButtonStyle = {
  ...baseButtonStyle,
  backgroundColor: '#8b2020',
  color: '#ffffff',
};

const sliderCardStyle = {
  marginTop: '8px',
  padding: '14px',
  borderRadius: '14px',
  background: 'rgba(255, 255, 255, 0.04)',
  border: '1px solid rgba(255, 255, 255, 0.08)',
};

const sliderStyle = {
  width: '100%',
  marginTop: '10px',
};

const stateMetaGridStyle = {
  display: 'grid',
  gridTemplateColumns: 'repeat(3, minmax(0, 1fr))',
  gap: '12px',
};

const smallInfoCardStyle = {
  background: 'rgba(255, 255, 255, 0.04)',
  borderRadius: '14px',
  padding: '12px 14px',
  border: '1px solid rgba(255, 255, 255, 0.08)',
};

const smallInfoLabelStyle = {
  fontSize: '12px',
  textTransform: 'uppercase',
  letterSpacing: '0.08em',
  opacity: 0.7,
};

const smallInfoValueStyle = {
  fontSize: '24px',
  fontWeight: 'bold',
  marginTop: '6px',
  fontFamily: 'MW Font, Arial, sans-serif',
};

const topTelemetryGridStyle = {
  display: 'grid',
  gridTemplateColumns: 'repeat(3, minmax(0, 1fr))',
  gap: '16px',
  alignItems: 'stretch',
};

const sideTelemetryStackStyle = {
  display: 'flex',
  flexDirection: 'column',
};

const metricCardLabelStyle = {
  fontSize: '13px',
  textTransform: 'uppercase',
  letterSpacing: '0.08em',
  opacity: 0.7,
};

const metricCardValueStyle = {
  fontSize: '34px',
  fontWeight: 'bold',
  marginTop: '10px',
  fontFamily: 'MW Font, Arial, sans-serif',
};

const temperatureBarChartStyle = {
  display: 'grid',
  gridTemplateColumns: 'repeat(12, minmax(0, 1fr))',
  gap: '10px',
  alignItems: 'end',
  minHeight: '320px',
};

const temperatureBarColumnStyle = {
  display: 'flex',
  flexDirection: 'column',
  alignItems: 'center',
  gap: '10px',
};

const temperatureBarTrackStyle = {
  width: '100%',
  height: '220px',
  display: 'flex',
  alignItems: 'flex-end',
  justifyContent: 'center',
  padding: '6px',
  background: 'linear-gradient(180deg, rgba(255, 255, 255, 0.02) 0%, rgba(255, 255, 255, 0.08) 100%)',
  border: '1px solid rgba(255, 255, 255, 0.08)',
  borderRadius: '14px',
};

const temperatureBarFillStyle = {
  width: '100%',
  borderRadius: '10px',
  background: 'linear-gradient(180deg, #73ff9d 0%, #f5cd19 55%, #ea4228 100%)',
  boxShadow: '0 8px 24px rgba(234, 66, 40, 0.18)',
};

const temperatureBarLabelStyle = {
  fontSize: '13px',
  textTransform: 'uppercase',
  letterSpacing: '0.06em',
  opacity: 0.72,
  textAlign: 'center',
};

const temperatureBarValueStyle = {
  fontSize: '16px',
  fontWeight: 'bold',
  fontFamily: 'MW Font, Arial, sans-serif',
  textAlign: 'center',
};

const simulationHintStyle = {
  marginTop: '10px',
  fontSize: '13px',
  lineHeight: 1.4,
  color: '#9fc3d9',
  textAlign: 'left',
};

const mediumGaugeStyle = {
  width: '100%',
  minHeight: '220px',
};

const mediumTickLabelStyle = {
  fontSize: '12px',
  color: '#ffffff',
  fontFamily: 'MW Font, Arial, sans-serif',
  fontWeight: 'bold',
};

const mediumValueLabelStyle = {
  fontSize: '24px',
  color: '#ffffff',
  fontFamily: 'MW Font, Arial, sans-serif',
  fontWeight: 'bold',
};

export default EngineDashboard;
