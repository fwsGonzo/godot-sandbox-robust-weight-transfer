# Readme

Install msys2 dependencies for Windows.

```bash
pacman -Sy mingw-w64-x86_64-riscv64-unknown-elf-gcc ninja cmake git
```

We want to see a compiler that we can use.

```bash
cd robust_weight_transfer/robust_skin_weight_transfer
./compile.sh
```

```bash
# Optional for debugging.
riscv64-unknown-elf-readelf.exe -a json_diff.elf
```
