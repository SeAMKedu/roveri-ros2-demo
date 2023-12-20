![hankkeen_logo](/images/roveri_logo.svg)

# ROS2-Demo

Robotic Operating System 2 (ROS2) on väliohjelmisto robottisovellusten kehittämiseen ja se koostuu avoimen lähdekoodin kirjastoista ja työkaluista. Alla on muutamia ROS2:seen liittyviä käsitteitä:
* Paketti (*engl. package*) on organisointiyksikkö ROS2-sovelluksen koodille. Paketin avulla omia robottisovelluksia voidaan jakaa toisille.
* Noodi (*engl. node*) on laskentaa suorittava prosessi, joka käyttää ROS2-väliohjelmistoa.
* Aihe (*engl. topic*) on kahden tai useamman noodin välinen viestintäväylä.

Lisätietoja ROS2:sta on saatavilla täältä: [https://docs.ros.org/](https://docs.ros.org/).

Tämä repository sisältää kaksi ROS2-pakettia. *gnssrecv*-paketin noodi lukee kerran sekunnissa u-bloxin satelliittivastaanottimen geodeettisen paikkaratkaisun ja julkaisee sen JSON-formaatissa *gnsspos*-aiheeseen. *agvdemo*-paketin noodi tilaa samaa aihetta ja tulostaa terminaaliin julkaistun paikkaratkaisun.

Noodeja voidaan ajaa sekä samalla tietokoneella...

![option1](/images/demo_option1.png)

...että kahdella eri tietokoneella, jotka on liitetty samaan verkkoon.

![option2](/images/demo_option2.png)

## Demon käyttäminen

### Esivaatimukset

Tietokoneelle tulee olla asennettu Ubuntu 22.04 Linux-jakelu ja ROS2 (Humble) -väliohjelmisto. ROS2:sen asennusohjeet on saatavilla alla olevasta linkistä:

[https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

ROS2-pakettien kääntämiseen käytetään *colcon*-komentorivityökalua. Se voidaan asentaa komennolla:

```
sudo apt install python3-colcon-common-extensions
```

Datan lukemiseen satelliittivastaanottimesta käytetään *pyubx2* ja *pyserial* Python-paketteja. Ne voidaan asentaa komennolla:

```
pip3 install -r requirements.txt
```

Lisäksi tarvitaan u-bloxin satelliittivastaanotin. Tässä demossa on käytetty u-blox C102-F9R -satelliittivastaanotinta, mutta datan lukeminen onnistunee myös u-blox C099-F9P -satelliittivastaanottimesta.

### Workspace-kansio

Omia ROS2-paketteja varten kannattaa luoda *workspace*-kansio.

[https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)

Workspace-kansio sisältää *build*, *install*, *log*, ja *src* alikansiot. Omien ROS2-pakettien lähdekoodi on *src*-alikansiossa.

### ROS2-pakettien kääntäminen

Omat ROS2-paketit käännetään (*engl. build*) workspace-kansion juuressa *colcon*-komentorivityökalun avulla

```
colcon build
```

Vinkki: Jos saat *SetuptoolsDeprecationWarning*-varoituksen käännöksen yhteydessä, päivitä *setuptools*-paketti vanhempaan versioon.

```
pip3 install setuptools==58.2.0
```

### Noodien ajaminen

Liitä satelliittivastaanotin tietokoneeseen ja odota muutama minuutti, että vastaanotin saa laskettua oman sijaintinsa.

*gnssrecv*-paketissa oleva noodi lukee sarjaportin kautta dataa satelliittivastaanottimesta, joten noodi täytyy ajaa *sudo*-oikeuksilla. Tämä tarkoittaa, että myös *agvdemo*-paketin noodi täytyy ajaa *sudo*-oikeuksilla, että se voi vastaanottaa viestejä.

Avaa terminaali ja aja seuraavat komennot:
```
$ sudo -i
[sudo] password for myUsername: 
cd /home/myUsername/myWorkspace
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run gnssrecv talker
```

Avaa uusi terminaali ja aja seuraavat komennot:
```
$ sudo -i
[sudo] password for myUsername: 
cd /home/myUsername/myWorkspace
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run agvdemo listener
```

Molemmat noodit voidaan sammuttaa painamalla Ctrl+c.

## Tekijätiedot

Hannu Hakalahti, Asiantuntija TKI, Seinäjoen ammattikorkeakoulu

## Hanketiedot

* Hankkeen nimi: Autonomiset ajoneuvot esiselvityshanke
* Rahoittaja: Töysän säästöpankkisäätiön tutkimusrahasto
* Aikataulu: 01.08.2023 - 31.06.2024
---
![rahoittajan_logo](/images/toysan-saastopankkisaatio-logo.jpg)

![seamk_logo](/images/SEAMK_vaaka_fi_en_RGB_1200x486.jpg)