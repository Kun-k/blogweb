1. 运行时提示`TypeError: unsupported operand type(s) for *: 'AsyncIOLoop' and 'float'`，报错位置为获取`client`连接的语句。

   解决方法为更新`msgpack-rpc-python `：

   ```
   pip install --upgrade msgpack-rpc-python 
   ```

2. 