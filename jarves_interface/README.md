Directions:

1. Install [RMS](http://wiki.ros.org/rms/Tutorials/InstallTheRMS)
2. Add a new Interface by following the [tutorial](http://wiki.ros.org/rms/Tutorials/AGuideToInterfaceDevelopment) named 'Jarves'
3. Make symlinks:
    1. `ln -s RMS_HOME_DRECTORY/app/Controller/JarvesInterfaceController.php path/to/jarves_interface/app/Controller/JarvesInterfaceController.php`
    2. `mkdir RMS_HOME_DIRECTORY/app/View/JarvesInterface && ln -s RMS_HOME_DIRECTORY/app/View/JarvesInterface/view.ctp /path/to/jarves_interface/app/View/JarvesInterface/view.ctp`
