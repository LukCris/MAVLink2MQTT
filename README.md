# MAVLink2MQTT

### Troubleshooting 
- Installare `future` se non è rilevato il modulo `past` nel file `__init__.py` di dronekit
- Modificare la classe Parameters nel file `__init__.py` di dronekit in:
  ```python
  class Parameters(collections.abc.MutableMapping, HasObservers):
  ```
