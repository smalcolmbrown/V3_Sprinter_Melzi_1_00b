# V3_Sprinter_Melzi_1_00b
The July 2017 Eaglemoss firmware update Plus minimal changes needed to add Error beep codes and echoing the error to the terminal and fixing bug in manage_heater() correcting error code generation in the event of a thermistor error  

New PC style beep codes to help diagnose thermistor errors.  
One long beep and one short beep extruder themistor low (open circuit themistor)  
One long beep and two short beeps heated bed themistor low (open circuit themistor)  
One long beep and three short beeps extruder themistor high (short circuit themistor)  
One long beep and four short beeps heated bed themistor high (short circuit themistor)  
