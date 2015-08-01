
//=========================================================
// Simple Usage
//=========================================================
In the openplatform/ top folder,

$ make config
$ make

The output oImage is located at out/<project>_<platform>/oImage.
You can use UsbDowner.exe to download the oImage to target.

To read more infomation about how to build and run the system,
please study the document in "doc/HOW_TO_BUILD.TXT"



//=========================================================
// Build Commands
//=========================================================
1. Build/Clean All
make
make clean

2. Build/Clean SDK
make sdk
make sdk_clean

3. Build/Clean Kernel
make kernel
make kernel_clean

4. Build/Clean Current Active Product (without kernel and SDK)
make product
make product_clean

5. Build/Clean Current Active Project
make project
make project_clean

6. Build/Clean Current Active Platform
make platform
make platform_clean

7. Build/Clean Other Product
make -C ./out/<product>
make -C ./out/<product> clean
	

	
//=========================================================
// Unpack From Kernel Release
//=========================================================

1. Put kernel_patch.tgz to sdk/os/

2. Get linux-2.6.32.9.tar.gz and put it to sdk/os/ folder

	http://www.kernel.org/pub/linux/kernel/v2.6/linux-2.6.32.9.tar.gz
		or 
	https://svn.general.com/tp/gp8kv/tools/linux/

3. In openplatform/ folder, use "make kernel_unpack" to start unpack flow

	$ make kernel_unpack

4. After all, openplatform/sdk/os/kernel-2.6.32 is the kernel source unpacked

	
	
