
# Indoor Air Quality System
Sistema per il monitoraggio della qualità dell'aria. Vengono monitorati CO, eCO2, TVOC, temperatura ed umidità


## Sensors
Il sistema è stato implementato su un arduino UNO.

Sono stati utilizzati
- ENS160 per la rilevazione della qualità dell'aria

- DHT11 per la rilavazione dell'umidità (sarebbe da sceglierne uno migliore)

- BMP180 per la rilevazione della temperatura

- Traffic light LED per la segnalazione della qualità dell'aria

- ZE15CO per la rilevazione di CO

- Buzzer passivo per la segnalazione sonora del peggioramento dell'aria

- Display TFT per la visualizzazione delle rilevazioni in modo testuale e grafico

Prossimamente verranno aggiunti i seguenti componenti:

- Pulsante per l'accesione del display a comando

- Interruttore per mutare il buzzer

- Scheda SD per la memorizzazioni dei dati per la renderizzazione del grafico

# Performance
Lo sketch viene prototipato su un Arduino UNO occupando il 96% della memoria Flash (max 32KB) e 87% di RAM (max 2KB) 

# Circuito