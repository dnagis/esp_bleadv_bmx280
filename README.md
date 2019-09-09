Mix de esp32_bmp280_i2c (mon mien)
et de 
	esp-idf/examples/bluetooth/ble_adv
	
pour connection i2c voir esp32_bmp280_i2c

récupère la température d'un bm\<x\>280 connecté en i2c à l'esp32 et
advertise le résultat pendant N secondes, puis dodo, remesure, readv...

adresse (0x77 ou 0x76) et numéros de pins définis dans 
	components/bmx280/include/bmx280.h

pour l'instant compatible avec Bluez-BLE -> lescan (attention au filter duplicates)


