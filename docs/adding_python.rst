=================================
Integracja węzłów w języku Python
=================================

W dokumencie tym wyjasnię w jaki sposób dodać węzły napisane w języku Python do istniejącego
już pakietu opartego o CMake.
Pierwsza sekcja poświęcona jest temu jak napisać prosty węzeł nadawcy w Pythonie i jeżeli
czytelnik ma już w tym temacie doświadczenie może ją pominąć.

Implementacja węzła nadawcy w języku Python
===========================================
Skrypt Pythona zawierający węzeł musi rozpoczynać się od takzwanej linii shebang::
    
    #!/usr/bin/env python3

linia ta wskazuje systemowi który interpreter ma być wykorzystany podczas wykonywania 
skryptu.
W przypadku czysto pythonowych pakietów ROS2 obecność tej linii nie jest wymagana ponieważ 
system zarządzania pakietami zajmuje się tą kwestią.
Natomiast w przypadku pakietów opartych o CMake trzeba pamiętać o jej dodaniu.
Żeby wykorzystać elementy ekosystemu ROS2 należy je zaimportować do naszego kodu::

    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

Biblioteka rclpy zapewnia dostęp do podstawowych funkcjonalności ROS2, jest to odpowiednik
biblioteki "rclcpp" z C++.
Dodatkowo importujemy klasy Node i String. Podobnie jak w przykładzie z C++ tutaj również 
nasza klasa węzła będzie musiała dziedziczyć po domyślnej.
Deklaracja klasy wygląda następująco

NAPISZ TO

Inicjacia i struktura pakietu
=============================
Pierwszym krokiem jest stworzenie pakietu opartego o CMake dokładnie tak jak w przypadku
gdybyśmy chcieli kożystać tylko z kodu C++::

    ros2 pkg create pytransmiter --build-type ament_cmake

W efekcie czego w katalogu pakietu pojawiają się następujące pliki:
* CMakeLists.txt
* package.xml

oraz następujące foldery:
* include
* src

Żeby być w stanie pisać też w Pythonie musimy ręcznie stworzyć dwa kolejne foldery:
* pytransmiter
* scripts

przy czym nazwa pierwszego folderu będzie zawsze odpowiadała nazwie całego pakietu.
Dodatkowo w folderze tym musi znaleźć się pusty plik "__init__.py", zakładając że znajdujemy się 
w głównym folderze pakietu możemy go stworzyć poleceniem::

    touch pytransmiter/__init__.py

W katalogu pytransmiter znajdować się będzie kod bibliotek i modółów które następnie 
będziemy mogli importować w wszystkich skryptach wykorzystujących ekosystem ROS2 i naszą
przestrzeń roboczą.
W katalogu "scripts" znajdować się będzie natomiast kod węzłów będących częścią tego pakietu.
Teraz możemy umieścić w tych katalogach przygotowane uprzednio skrypty, do folderu 
pytransmiter należy skopiować helpers.py a do scripts pypublisher.py.

Modyfikacja opisu pakietu (package.xml)
=======================================
Do listy wymaganych narzędzi do budowania pakietu należy dodać rozszerzenie do cmake 
umożliwiające pracę z Pythonem::

    <buildtool_depend>ament_cmake_python</buildtool_depend>

wymagane jest też dodanie zależności od biblioteki rclpy::

    <depend>rclpy</depend>

Obydwie te zależności należy wprowadzić w przeznaczonych do tego częściach pliku tak że 
po modyfikacjach plik package.xml będzie wyglądał następująco::

    <?xml version="1.0"?>
    <?xml-model href="http://download.ros.org/schema/package_format3.xsd"
                schematypens="http://www.w3.org/2001/XMLSchema"?>
    <package format="3">
    <name>pytransmiter</name>
    <version>0.0.1</version>
    <description>TODO: Package description</description>
    <maintainer email="your@email.com">Name</maintainer>
    <license>TODO: License declaration</license>
    <buildtool_depend>ament_cmake</buildtool_depend>
    <buildtool_depend>ament_cmake_python</buildtool_depend>
    <depend>rclcpp</depend>
    <depend>rclpy</depend>
    <test_depend>ament_lint_auto</test_depend>
    <test_depend>ament_lint_common</test_depend>
    <export>
    <build_type>ament_cmake</build_type>
    </export>
    </package>

Modyfikacja plku CMakeLists
===========================
W pliku CMakeLists musimy podobnie jak w opisie pakietu zaznaczyć zależność od pakietów
ament_cmake_python oraz rclpy. Tutaj robimy to dodając następujące linie::

    find_package(ament_cmake_python REQUIRED)
    find_package(rclpy REQUIRED)

Oczywiście należy to zrobić w tym samym miejscu w którym deklarujemy pozostałe zależności
za pomocą dyrektywy "find_package".
Następnie zaraz za częścią pliku poświęconą instalowaniu plików binarnych będących rezultatem
kompilacji kodu C++ należy dodać część instalującą skrypty pythonowe do odpowiednich lokacji::

    ament_python_install_package(${PROJECT_NAME})
    # Following part must include all scripts with nodes.
    install(PROGRAMS
    scripts/pypublisher.py
    DESTINATION lib/${PROJECT_NAME}
    )

Kompilacja
==========
Sam proces kompilacji pakietu nie różni się od tego jak wygląda to dla wariantu zawierającego
jedynie kod C++::

    colcon build --packages-select pytransmiter

oczywiście skrypty pythonowe nie są kompilowane a jedynie kopiowane do ścieżek w których
mogą zostać wywołane przez polecenie ros2 run oraz skrypty launch.
Należy zauważyć że uruchamiane są kopie skryptów z katalogu "scripts" a nie one same, 
dlate rozdział na "źródła" i "pliki wykonywalne" jest zachowany też dla kodu w Pythonie.
Oznacza to że jakiekolwiek zmiany wprowadzone w kodzie w katalogu źródłowym pakietu nie będą
miały wpływu na węzły uruchamiane w środowisku ROS2 do kiedy pakiet ten nie zostanie ponownie 
skompilowany.
Istnieje możliwość obejścia tego wywołując kompilację z opcją --symlink-install, spowoduje to 
umieszczenie w ścieżkach ROS2 łączy prowadzących bezpośrednio do plików w katalogu źródłowym
projektu.
Przy wyborze takiego rozwiązania należy pamiętać o dodaniu uprawnień do wykonywania 
skryptów.
