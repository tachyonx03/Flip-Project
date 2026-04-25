### Disturbance Injection                                                     
  - Torque disturbance applied via `nonlinearAirframe/addTorqueDisturbance.m`
  - Disturbance block connected directly to the nonlinear airframe model in     
  Simulink        
                                                                                
  ```matlab       
  % Constant torque
  setTorqueDisturbance('constant', 'roll', 0.001)                               
   
  % Step (t=5s ~ t=10s)                                                         
  setTorqueDisturbance('step', 'roll', 0.002, 5, 10)
                                                                                
  % Sinusoidal disturbance 2Hz
  setTorqueDisturbance('sine', 'roll', 0.001, 2)                                
                                                                                
  % Disturbance off
  setTorqueDisturbance('off')                
