gcs:send_text(0, "Starte einmalige I2C-Überprüfung...")

-- Versuche, das I2C-Modul zu initialisieren
local i2c = I2C.init(0, {speed = 100000}) -- I2C-Bus 0 mit 100 kHz initialisieren

if i2c then
    gcs:send_text(0, "I2C-Interface ist verfügbar.")
else
    gcs:send_text(0, "I2C-Interface ist NICHT verfügbar.")
end

-- Skript beendet sich hier
