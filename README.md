# uros_pico_exemples

This repository contains a collection of **micro-ROS** examples for the **Raspberry Pi Pico**, using the [Pico SDK](https://github.com/raspberrypi/pico-sdk).

Each folder is an independent example.

---

## Prerequisites

- [Pico SDK](https://github.com/raspberrypi/pico-sdk) installed and configured properly.
- micro-ROS [installed](https://robotcopper.github.io/micro-ROS/).

---

## Building and Uploading

To build and upload an example to your Raspberry Pi Pico:

1. Open a terminal in the folder of the example you want to use.
2. Create a `build` directory and navigate into it:

   ```bash
   mkdir build
   cd build

3. Generate build files with CMake:

   ```bash
   cmake ..
   ```

4. Compile the project:

   ```bash
   make
   ```

5. Connect your Pico to your computer while holding the BOOTSEL button to mount it as a USB drive. (Or you could use picotool.)

6. Copy the generated `.uf2` file from the `build` folder to the Pico.

---

## License

This project is licensed under the [BSD 3-Clause License](LICENSE).

