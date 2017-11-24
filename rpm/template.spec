Name:           ros-kinetic-qb-move-hardware-interface
Version:        1.0.6
Release:        0%{?dist}
Summary:        ROS qb_move_hardware_interface package

Group:          Development/Libraries
License:        BSD 3-Clause
URL:            http://wiki.ros.org/qb_move_hardware_interface
Source0:        %{name}-%{version}.tar.gz

Requires:       ros-kinetic-control-toolbox
Requires:       ros-kinetic-qb-device-hardware-interface
Requires:       ros-kinetic-roscpp
Requires:       ros-kinetic-transmission-interface
BuildRequires:  ros-kinetic-catkin
BuildRequires:  ros-kinetic-control-toolbox
BuildRequires:  ros-kinetic-qb-device-hardware-interface
BuildRequires:  ros-kinetic-roscpp
BuildRequires:  ros-kinetic-transmission-interface

%description
This package contains the hardware interface for qbrobotics® qbmove device.

%prep
%setup -q

%build
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree that was dropped by catkin, and source it.  It will
# set things like CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/kinetic/setup.sh" ]; then . "/opt/ros/kinetic/setup.sh"; fi
mkdir -p obj-%{_target_platform} && cd obj-%{_target_platform}
%cmake .. \
        -UINCLUDE_INSTALL_DIR \
        -ULIB_INSTALL_DIR \
        -USYSCONF_INSTALL_DIR \
        -USHARE_INSTALL_PREFIX \
        -ULIB_SUFFIX \
        -DCMAKE_INSTALL_LIBDIR="lib" \
        -DCMAKE_INSTALL_PREFIX="/opt/ros/kinetic" \
        -DCMAKE_PREFIX_PATH="/opt/ros/kinetic" \
        -DSETUPTOOLS_DEB_LAYOUT=OFF \
        -DCATKIN_BUILD_BINARY_PACKAGE="1" \

make %{?_smp_mflags}

%install
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree that was dropped by catkin, and source it.  It will
# set things like CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/kinetic/setup.sh" ]; then . "/opt/ros/kinetic/setup.sh"; fi
cd obj-%{_target_platform}
make %{?_smp_mflags} install DESTDIR=%{buildroot}

%files
/opt/ros/kinetic

%changelog
* Fri Nov 24 2017 Alessandro Tondo <alessandro.tondo@qbrobotics.com> - 1.0.6-0
- Autogenerated by Bloom

* Tue Jun 27 2017 Alessandro Tondo <alessandro.tondo@qbrobotics.com> - 1.0.5-0
- Autogenerated by Bloom

* Fri Jun 23 2017 Alessandro Tondo <alessandro.tondo@qbrobotics.com> - 1.0.4-0
- Autogenerated by Bloom

* Mon Jun 19 2017 Alessandro Tondo <alessandro.tondo@qbrobotics.com> - 1.0.1-0
- Autogenerated by Bloom

