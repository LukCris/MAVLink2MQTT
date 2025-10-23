# MAVLink2MQTT

### Troubleshooting 
- Modificare la classe Parameters nel file `__init__.py` di dronekit in:
  ```python
  class Parameters(collections.abc.MutableMapping, HasObservers):
  ```
