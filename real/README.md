compile with lgpoid and lpthread
```
 g++ stepper_driver/cli_control.cpp -o cli_control -lgpiod -lpthread && ./cli_control
```
```
 g++ stepper_driver/rand_agent.cpp -o random_agent -lgpiod -lpthread && ./random_agent
```

https://www.ti.com/lit/an/sloa293a/sloa293a.pdf?ts=1757748698589

```
wget https://github.com/microsoft/onnxruntime/releases/download/v1.20.1/onnxruntime-linux-aarch64-1.20.1.tgz
tar -xvzf onnxruntime-linux-aarch64-1.20.1.tgz
```


```
mkdir build && cd build
cmake ..
make
```

```
mkdir build
cd build
cmake ..
make -j4
```