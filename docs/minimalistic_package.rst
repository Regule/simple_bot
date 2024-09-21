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


Kod źródłowy węzła
==================
W celu stworznia wezła musimy umieścić jego kod źródłowy w katalogu src a następnie 
zaktualizować plik CMakeLists.txt żeby uwzględniał jego istnienie.
W naszym przykładzie nazwiemy ten plik tiny_node.cpp.
Najmiejszy kod który jest w stanie funkcjonować jako węzeł ROS2 wygląda następująco::

    #include "rclcpp/rclcpp.hpp"

    int main(int argc, char **argv) {
        rclcpp::init(argc, argv);
        auto node = rclcpp::Node::make_shared("tiny_node");
        rclcpp::spin(node);
        rclcpp::shutdown();
        return 0;
    }

Pierwsza linia załącza nagłowek zawierający podstawowe funkcjonalności ROS2, znajdują się 
one w przestrzeni nazw "rclcpp".
Pierwsza funkcja::
    
    rclcpp::init(argc, argv);

incjalizuje globalny kontekts ROS2 dla danego wątku. 
Kontekst ten jest niezbędny dla większości pozostałych funkcjonalności w tym możliwości 
tworzenia węzłow. Dodatkowo zapewnia on obsłógę sygnałów co umożliwia przerwanie działania 
programu za pomocą kombinacji Ctrl+C.

Następnie za pomocą linii::

    auto node = rclcpp::Node::make_shared("tiny_node");

tworzymy obiekt węzła.
W tym przypadku używamy bezpośrednio klasy Node, tak naprawdę w rzeczywistych zastosowaniach
będziemy zawsze kożystać z wyspecjalizowanych klas dziedziczących po Node ponieważ sama klasa
Node zapewnia jedynie najbardziej podstwowe funkcjonalności węzła.
Metoda statyczna "make_shared" obudowuje "make_shared<Type>(constructor_arguments)" i umożliwia
bardziej wygodne tworzenie wskaźników do węzłów.

Następnie na obiekcie węzła zostaje wywołana funkcja::

    rclcpp::spin(node);

która uruchamia węzeł.
W tej chwili 
