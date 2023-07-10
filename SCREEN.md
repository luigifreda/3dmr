# Screen 

A brief list of the main screen commands. 

## Check the active sessions 

`$ screen -list`


## Connect to an active session 

`$ screen -R <session_name>`


## Once connected to a session 

* detach from that session: first, press `CTRL+A`, then `D`

* kill that session: first, press `CTRL+A`, then `K`

* scroll with arrows: 
  - first, press `CTRL+A`, then `Esc`.
  - next, press the `Up` and `Down` arrow keys or the `PgUp` and `PgDn` keys to scroll through previous output
  - press `Esc` to exit scrollback mode.
  
## Kill all the active screen sessions 

`$ killall screen; screen -wipe`

Indeed, the command `screen -wipe` is used to clean the screen list 


## Enable scrolling in your screen session 

Create a file `~/.screenrc` and add the following lines therein 
```
# Enable mouse scrolling and scroll bar history scrolling
termcapinfo xterm* ti@:te@
``` 


## Enable logging in your screen session 

Create a file `~/.screenrc` and add the following lines therein 
```
logfile ${HOME}/.ros/3dmr/logs/%H_%S_%Y%m%d-%c.log
logfile flush 1
log on

# Uncomment to enable timestamping in logs
logtstamp after 1
# logtstamp string "[ %t: %Y-%m-%d %c:%s ]\012"
logtstamp on
```

the logs will be created in the folder `${HOME}/.ros/3dmr/logs`. This log folder is set in the file `config.sh` (main folder of the repo).
