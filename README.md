# Mix esp32_bmp280_i2c - esp-idf/examples/bluetooth/ble_adv
	
Pour connections i2c voir esp32_bmp280_i2c

Adresse Register (0x77 ou 0x76 -> selon bm**x** ou bm**e**) et numéros de pins pour connexion définis dans 
	components/bmx280/include/bmx280.h

Récupère la température d'un bm**x**280 connecté en i2c à l'esp32 et
advertise le résultat pendant n secondes, puis dodo, remesure, readv...

Compatible avec Bluez-BLE -> lescan (attention au filter duplicates)


