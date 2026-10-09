# ina_logger — логгер датчика тока INA219 + температур SoC

Пишет строку 4 раза в секунду в `/home/sg/ina/ina-YYYYMMDD.log`:

    2026-10-09 15:08:55.096 up=349.4   7.692 V   +0.0536 A  T=79.9/78.6/77.1/76.5

- V — напряжение шины серв (INA219, 0x41, шунт 0.1 Ом), A — ток шины (знак = направление),
  T — температуры зон (gpu / ve / cpu / ddr). fsync раз в секунду — переживает пропадание питания.

## Установка на банане

    scp ina_logger.py sg@<pi>:/home/sg/
    scp ina-logger.service sg@<pi>:/home/sg/
    ssh sg@<pi>
    sudo cp /home/sg/ina-logger.service /etc/systemd/system/
    sudo systemctl daemon-reload && sudo systemctl enable --now ina-logger

## Эксплуатация

    systemctl status ina-logger        # состояние
    tail -f /home/sg/ina/ina-*.log     # живой лог
    sudo systemctl disable --now ina-logger   # выключить

Живой лог нужен для расследований «сама вырубилась»: всплеск температуры
перед выключением виден в последних строках (штатная тепловая защита
пишет в журнал `kernel: thermal … critical temperature reached`).
