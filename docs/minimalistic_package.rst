=================
Tworzenie pakietu
=================

W tym dokumencie opiszę w jaki sposób stworzyć najprostszy pakiet ROS2 w języku C++.
Będzie on zawierał pojedynczy węzeł który będzie można uruchomić ale nie będzie on robił nic
ciekawego.
Jeżeli nie umieszczono odpowiedniej komendy w pliku bashrc należy pamiętać żeby przed 
wykonwyaniem jakichkolwiek innych czynności związanych z środowiskiem ROS2 wywołać w terminalu
polecenie::
    
    source /opt/ros/jazzy/setup.bash


Inicjalizacja pakietu
=====================

Zanim rozpoczniemy tworzenie konkertnego pakietu musimy stworzyć folder przestrzeni roboczej.
W moim przypadku nazwę ten katalog "ros_ws" i kiedy w przyszłości będę używał tej nazwy 
będzie mi chodziło właśnie o katalog-korzeń przestrzeni roboczej.
Oczywiście każdy może nazwać sobie ten katalog według własnego uznania, inaczej ma się 
sytuacja z katalogami umieszczonymi już w tej przestrzeni roboczej.

Manualnie musimy stworzyć jedynie katalog "src" w którym znajdować się będą katalogi z źródłami
pakietów. Pozostała struktura przestrzeni roboczej zostanie stworzona wraz z pierwszą kompilacją
źródeł w niej wykonaną.

Ponieważ każdy pakiet ma specyficzną strukturę więc żeby ułatwić jego tworzenie ROS2 zapewnia 
nam polecenie tworzące wszystkie najbardziej typowe elementy pakietu dzięki czemu unikamy
powtarzalnych czynności.
Żeby utworzyć pakiet w ten sposób należy przejść do katalogu z źródłami, czyli w moim przypadku
ros2/src, a następnie wywołać polecenie::

    ros2 pkg create --build-type ament_cmake --license Apache-2.0 minimal_node

W parametrach "ament_cmake" oznacza że będzie to pakiet oparty o CMake, alternatywą jest 
"ament_python" ale tego wariantu nie będziemy omawiać w naszym kursie. 
Dodatkowo można domyślnie zadeklarować rodzaj licencji, ostatnim parametrem jest nazwa węzła.

Tak utworzony pakiet zawierał będzie trzy pliki
* package.xml - informacje dla ROS2 odnośnie pakietu
* CMakeLists.txt - plik dla CMake informujący o źródłach, folderach docelowych oraz zależnościach
* LICENSE - treść licencji oprogramowania, w tym przypadku Apache-2.0

Większość zmian w pakiecie będzie wymagała aktualizacji zarówno package.xml jak i CMakeLists.txt.
Poza tymi plikami w katalogu projektu znajdują się następujące foldery 
* include - z jednym podfloderem którego nazwa odpowiada nazwie pakietu, tutaj znajdą się pliki
            nagłówkowe umożliwiające innym pakietom korzystanie z jego zasobów.
* src - folder z plikami źródłowymi
