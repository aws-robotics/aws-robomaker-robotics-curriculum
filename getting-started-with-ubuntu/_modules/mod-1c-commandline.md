---
title: Command line
permalink: /modules/1/commandline
---

Let's see some useful commands in the commandline in Ubuntu, which will be useful to navigate the filesystem in the computer of a robot or in a server in the cloud. Please refer for complete information to {% cite cooper2014advanced %}.

Open a new `Terminal` from the list of applications. 

![Open Terminal](/img/terminal.png)

A new window will appear with a "prompt", which is from the "shell", a program that interprets the commands written and runs programs corresponding to each command. The text will be the following

    class@ubuntu:~$

The main information included in that line is as follows `username@hostname:current_directory`.

## Navigate files

Any OS provides a file system for information storage and retrieval. A typical structure is depicted in the following figure.

<a title="Ppgardne / CC BY-SA (https://creativecommons.org/licenses/by-sa/4.0)" href="https://commons.wikimedia.org/wiki/File:Standard-unix-filesystem-hierarchy.svg"><img width="512" alt="Standard-unix-filesystem-hierarchy" src="https://upload.wikimedia.org/wikipedia/commons/thumb/f/f3/Standard-unix-filesystem-hierarchy.svg/512px-Standard-unix-filesystem-hierarchy.svg.png"></a>

The filesystem is a tree, where the root is denoted by `/`. There are two ways to access to files:
- **absolute path**, which is defined specifying the location of a file starting from the root.
- **relative path**, which is defined specifying the location from the current folder.
The concept of absolute and relative path is important as it applies to all commands below.

`ls` (list) is a command to display the content of a folder. It can take as argument a path to a folder.

For example, the following command will display the content of the current location (relative path):

    class@ubuntu:~$ ls ./
    Desktop    Downloads         Music     Public     Videos
    Documents  examples.desktop  Pictures  Templates

Typing `pwd` (print working directory) will display the current working directory.

    class@ubuntu:~$ pwd
    /home/class

The following command will display the content of the current location (absolute path):

    class@ubuntu:~$ ls /home/class
    Desktop    Downloads         Music     Public     Videos
    Documents  examples.desktop  Pictures  Templates

Some files are hidden and can be displayed by using the option `-a`, i.e., 

    class@ubuntu:~$ ls -a
    .   .bash_history  .bashrc  .config  Documents  examples.desktop  .ICEauthority  .mozilla  .npm      .profile  .ssh                       Templates
    ..  .bash_logout   .cache   Desktop  Downloads  .gnupg            .local         Music     Pictures  Public    .sudo_as_admin_successful  Videos
    class@ubuntu:~$

Of particular importance is the file `.bashrc`, which is a script that is run every time a terminal (bash) is started -- we will see it in the ROS installation. 

Note that there are a few special characters indicating specific directories:
- `~` is used to identify the home directory -- in the example, it is `/home/class`.
- `.` indicates the current directory -- in the example, again it is `/home/class`.
- `..` indicates the parent of the current directory -- in the example that would be `/home`.


A number of other commands useful to modify files and directories:

- Create a directory with `mkdir` (make directory): `mkdir [option] dir_name`. E.g., to create a folder named `catkin_ws` in the current location:

```
class@ubuntu:~/$ mkdir catkin_ws
```


- Go to a different directory with `cd` (change directory): `cd path`. E.g., to access the folder `catkin_ws` (relative path) just created:

```
    class@ubuntu:~$ cd catkin_ws
    class@ubuntu:~/catkin_ws$
```

Note that typing just `cd` will return to the home directory.

- Rename an existing directory or file with `mv` (move): `mv original_dir_or_file renamed_dir_or_file`. E.g., to rename the folder previously created `catkin_ws` to `ros_catkin_ws`
    class@ubuntu:~/$ mv catkin_ws ros_catkin_ws


- Remove files and directories with `rm` (remove):

    - For a single file: `rm file`
    - For a directory and all its content: `rm -r directory`

- Copy a file to a path with `cp` (copy): `cp file1 file2`

- Find files in `/home/class` that start with "prefix". `find /home/class/ -name 'prefix*'`

The filesystem provides a method for assigning permissions to specific users, groups of users, and all others. For example, typing `ls -l` in the home directory shows:

    drwxr-xr-x 2 class class 4096 Oct  4 11:05 Desktop
    drwxr-xr-x 2 class class 4096 Oct  4 11:05 Documents
    drwxr-xr-x 2 class class 4096 Oct  4 11:05 Downloads
    -rw-r--r-- 1 class class 8980 Oct  4 10:57 examples.desktop
    drwxr-xr-x 2 class class 4096 Oct  4 11:05 Music
    drwxr-xr-x 2 class class 4096 Oct  4 11:49 Pictures
    drwxr-xr-x 2 class class 4096 Oct  4 11:05 Public
    drwxr-xr-x 2 class class 4096 Oct  4 11:05 Templates
    drwxr-xr-x 2 class class 4096 Oct  4 11:05 Videos
    
The first letter `d` or `-` refers to whether it is a directory or a file, respectively. Then, `r` refers to reading permission, `w` writing permission, `x` execution permission, and `-` if no permission is granted. The first triplet refers to the user `class`, the second triplet to the group `class`, the last triplet to all others.

To change permissions of files and directories, `chmod` (change mode) can be used. There are different ways to do that. The more intuitive way is by directly using the letters. For example,

    chmod u+rwx file

gives the user all permissions, while

    chmod u-rwx file

removes the user all permissions.



## View and search in files
Let's download a file example with the command `wget`. 

    class@ubuntu:~$ wget https://loremipsum.de/downloads/original.txt

Text files can be viewed with `less`: `less filepath`. For example:

    class@ubuntu:~$ less original.txt

If it is long, it will scroll off the terminal.

which allows for scrolling over the content of the file. To exit from the view mode, press the 'q' character.

To search for a pattern in a file `grep pattern file`. This can be done also recursively in a directory `grep -r pattern directory`.

Note that `pattern` can be a word or a regular expression, which is a sequence of characters defining a search pattern. A complete tutorial is in {% cite goyvaerts2006regular %}.

## Edit files
While we are generally used to a graphical text editor, a number of situations when working with a robot will require the use of a text editor from command line. There are a number of them available. Some of the more popular ones include `vim` and `emacs`. The one referred here is `nano`, which is not as powerful as the other two mentioned, but is pre-installed for example in Ubuntu.

![Nano](/img/nano.png)

The caret symbol '^' represents the control key. For example, to exit `nano`, ctrl-X needs to be pressed.



## Getting help from the manual

`man` (manual) followed by the command for which more information is requested can be used to get its detailed syntax.

For example, `man ls` produces

    LS(1)                            User Commands                           LS(1)
    
    NAME
           ls - list directory contents
    
    SYNOPSIS
           ls [OPTION]... [FILE]...
    
    DESCRIPTION
           List  information  about  the FILEs (the current directory by default).
           Sort entries alphabetically if none of -cftuvSUX nor --sort  is  speci‐
           fied.
    
           Mandatory  arguments  to  long  options are mandatory for short options
           too.
    
           -a, --all
                  do not ignore entries starting with .
    
           -A, --almost-all
                  do not list implied . and ..
    
    
        ... and a lot more

To exit the view, press the key 'q'.

## Installing packages and libraries

It is useful to install new software and libraries on a new system. In many instances, this is possible using the package manager available in Ubuntu, based on `apt`, which allows to install, remove, or upgrade software packages.

In particular, the following commands are used:
- `sudo apt update` to update the list of available packages.
- `sudo apt upgrade` to upgrade the installed packages.
- `sudo apt install <package-name>` to install a package. E.g., to install `htop`, an interactive viewer for processes.


    sudo apt install htop

Note that `sudo` is necessary to run a command with administrative privileges (root user).

## Monitoring running processes

Any computer has executing programs running as the computer is on. They are called processes identified by a unique PID (process identifier). It is important to see the resources used, especially in a limited computational system such as the one on typical robots.

First install the interactive viewer:

    sudo apt install htop

The interactive viewer installed above can be executed with

    htop

<a title="PER9000, Per Erik Strandberg / CC BY-SA (http://creativecommons.org/licenses/by-sa/3.0/)" href="https://commons.wikimedia.org/wiki/File:Htop.png"><img width="512" alt="Htop" src="https://upload.wikimedia.org/wikipedia/commons/b/b1/Htop.png"></a>

Some of the information which is important to keep track of in robots and in general is the CPU% and MEM% usage.





Now that we have seen a very brief intro to Linux and the Terminal commands, let's assess these main commands before getting ready with ROS in the [next unit]().


{% capture ref %}
{% bibliography --cited %}
{% endcapture %}
{% include references.html ref=ref %}
