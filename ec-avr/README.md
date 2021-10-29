## compiling the code

For compiling and flashing, you need:
```
sudo apt install gcc-avr avr-libc avrdude
```

After compile with:
```
make
```

If you are using a new AVR, you first have to flash the fuse for the correct oscillator settings:
```
make fuse
```

Afterwards, flash the `.hex`-file.

```
make flash
```

