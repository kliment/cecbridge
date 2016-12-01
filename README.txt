USB to CEC command parser for stm32f042

Connect via usb-cec serial at 115200

Commands and responses:
Each command is one opcode character followed by zero or more parameter characters. Each command is terminated with \r, \n, or \r\n
Responses are terminated with \r\n

-> means sent to mcu
<- means response from mcu
do not actually send these characters

Echo:

-> E
<- E 

You should send E\n until you receive an E back when you first open a connection to synchronize with the device state.



Set local address:

-> L3
<- L3 (if address is valid)
-> Lx
<- FP (parameter error)



Transmit

-> Tf4 
(transmit from address f to address 4)
<- Tf4
(header plus zero bytes transmitted)
<- FN
(NAK)
<- FX
(transmit error)
<- FS
(source address error)
<- FD
(dest address error)
<- FC
(illegal separator character (should be ':'))
<- FB
(illegal message byte)
-> TF0:64:00:48:65:6c:6c:6f:20:77:6f:72:6c:64
<- TF0:64:00:48:65:6C:6C:6F:20:77:6F:72:6C:64 
(message transmitted successfully)



Receive:

<- RF0:64:00:48:65:6C:6C:6F:20:77:6F:72:6C:64
(message received)

<- F
(receive error)



Set promiscuous mode (listen to all messages):

-> P1
<- P1
(promiscuous mode enabled)
-> P0
<- P0
(promiscuous mode disabled)
-> P7
<- FP
(parameter error)



