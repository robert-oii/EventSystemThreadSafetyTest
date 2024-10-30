# Event System Thread Safety Test

This is a stripped down copy of the OCS event system (copy/pasted with some quick mods to get it to compile).

The intent is to allow using race detection tools to check for data races when making calls into the OCS event system concurrently from more than one thread.

More on data races:
- https://confluence.oii.oceaneering.com/pages/viewpage.action?pageId=181078148

Some tools we can use to detect data races:
- Thread Sanitizer (compile/link with `-fsanitize=thread`)
  - https://github.com/google/sanitizers/wiki/threadsanitizercppmanual
- Helgrind (`valgrind --tool=helgrind ./my_program`)

It's easier to work with this smaller example code than full ORCA/OCS:
- Building full ORCA/OCS with Thread Sanitizer is difficult (unsuccessful so far, though it should be possible with more work).
- Full OCS can be run with Helgrind, but it's easier to run Helgrind and read the results with this smaller example.

## Install prereqs
```Bash
sudo apt update
sudo apt install -y valgrind libtsan*
```

## Build
```Bash
cd ~
git clone https://github.com/robert-oii/EventSystemThreadSafetyTest

# Build with Thread Sanitizer:
mkdir ~/EventSystemThreadSafetyTest/build_with_tsan && cd $_
cmake .. -DSANITIZE=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build .

# To run in Helgrind, must build without Thread Sanitizer:
mkdir ~/EventSystemThreadSafetyTest/build_no_tsan && cd $_
cmake .. -DSANITIZE=OFF -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build .
```
## Run
### With Thread Sanitizer
```Bash
# In Ubuntu 22.04, gcc 11.4, Thread Sanitizer crashes:
#   FATAL: ThreadSanitizer: unexpected memory mapping 0x5b9716dfc000-0x5b9716dfe000
# Workaround: https://github.com/google/sanitizers/issues/1716
#   Warning: This disables ASLR, so only do this if you are willing to
#   lower the security of your system:
setarch `uname -m` --addr-no-randomize

# Run:
cd ~/EventSystemThreadSafetyTest/build_with_tsan
./events_thread_test
```
### In Helgrind
```Bash
cd ~/EventSystemThreadSafetyTest/build_no_tsan
valgrind --tool=helgrind ./events_thread_test
```
