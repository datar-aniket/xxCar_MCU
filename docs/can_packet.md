The custom and standard VESC CAN message definitions implemented in the workspace driver node

 are detailed below.

1. CAN Identification & ID Layout (VESC id 0x4d)

Frame Type: 29-bit Extended CAN Identifier (CAN_EFF_FLAG)
29-Bit Bit Allocation:(Packet_ID << 8) | Controller_ID
Packet ID:(can_id >> 8) & 0xFF
Controller ID:can_id & 0xFF(VESC Target Node ID)



2. Custom Combined Command Frames (Host $\rightarrow$ VESC)
CAN_PACKET_SET_CURRENT_SERVO (ID 69 / 0x45)
Custom single-frame packet to transmit motor current and servo position simultaneously, halving CAN bus latency.

DLC: 6 Bytes
Payload Layout (Big-Endian):
Bytes 0–3:int32_t– Motor Current in Amps $\times 1000$ (e.g., $5.0,\text{A} \rightarrow 5000$)
Bytes 4–5:int16_t– Steering Servo Pulse Width in microseconds ($\mu\text{s}$, range: $800$ to $2200,\mu\text{s}$)

Implementation:

CAN_PACKET_SET_DUTY_SERVO (ID 70 / 0x46)
Custom single-frame packet to transmit motor duty cycle and servo position simultaneously.

DLC: 6 Bytes
Payload Layout (Big-Endian):
Bytes 0–3:int32_t– Duty Cycle Ratio $\times 100,000$ (e.g., $0.05 \rightarrow 5000$)
Bytes 4–5:int16_t– Steering Servo Pulse Width in microseconds ($\mu\text{s}$, range: $800$ to $2200,\mu\text{s}$)

Implementation:



3. Extended Telemetry Frame (VESC $\rightarrow$ Host)
CAN_PACKET_STATUS_5 (ID 27 / 0x1B)
Telemetry packet received from the VESC controller containing tachometer data, filtered motor current, and ADC voltage.

DLC: 8 Bytes
Payload Layout (Big-Endian):
Bytes 0–3:int32_t– Tachometer Count (accumulated motor position counts)
Bytes 4–5:int16_t– Filtered Total Current in Amps divided by $10.0$ ($I = \text{raw} / 10.0$)
Bytes 6–7:int16_t– External ADC1 Input Voltage divided by $1000.0$ ($V_{\text{adc}} = \text{raw} / 1000.0$, used for steering feedback)

Decoder Implementation:



4. Standard VESC Command Frames Used
CAN_PACKET_SET_DUTY (ID 0 / 0x00)

DLC: 4 Bytes
Payload Layout (Big-Endian):
Bytes 0–3:int32_t– Duty Cycle Ratio $\times 100,000$

Implementation:

CAN_PACKET_SET_CURRENT (ID 1 / 0x01)

DLC: 4 Bytes
Payload Layout (Big-Endian):
Bytes 0–3:int32_t– Motor Current in Amps $\times 1000$

Implementation:

CAN_PACKET_PROCESS_SHORT_BUFFER (ID 8 / 0x08)

DLC: 5 Bytes
Payload Layout:
Byte 0:0xFF(Host Sender ID)
Byte 1:0x02(Process Flag)
Byte 2:0x0C(COMM_SET_SERVO_POS)
Bytes 3–4:int16_t(Big-Endian) – Servo Pulse Width in microseconds ($\mu\text{s}$)