#include <stdio.h>
#include <SFML/Graphics.hpp>
#include <SFML/Graphics/Drawable.hpp>
#include <SFML/Graphics/Transformable.hpp>


#include "libmodule.h"
int main(int argc, char *argv[]) {
  printf("Project New_Project\n");

  // setup font
  sf::Font font;
  if (!font.loadFromFile("resource/fonts/NanumGothicCoding-Regular.ttf"))
  {
    printf("\033[1;31m[%s][%d] :x: Font Load Error \033[m\n",
        __FUNCTION__,__LINE__);
    exit(1);
  }
  sf::Texture Texture00;
  if (!Texture00.loadFromFile("resource/pics/SFML_ex.png"))
  {
    exit(1);
  }
 

  sf::RenderWindow window(sf::VideoMode(800, 600), "SFML Window");
  // Set framerate 
  window.setFramerateLimit(60);
 
  // Text
  sf::Text Text00;
  Text00.setFont(font);
  Text00.setString("Text Test: Press Q , to exit");
  Text00.setStyle(sf::Text::Bold | sf::Text::Underlined);
  // Circle
  sf::CircleShape Circle00(50);
  Circle00.setPosition(100, 100);
  Circle00.setFillColor(sf::Color(0,255,0,128));
  // Rectangle
  sf::RectangleShape Rect00(sf::Vector2f(80, 20));
  Rect00.setPosition(0, 60);
  // Sprite
  sf::Sprite Sprite00(Texture00);
  
  Sprite00.setPosition(200,50);


  // run the program as long as the window is open
  while (window.isOpen())
  {
    sf::Event event;
    // event handler
    while (window.pollEvent(event))
    {
      // "close requested" event: we close the window
      if (event.type == sf::Event::Closed)
        window.close();
      if (event.type == sf::Event::GainedFocus) {
        printf("\033[1;33m[%s][%d] :x: focus in\033[m\n",__FUNCTION__,__LINE__);
      } 
      if (event.type == sf::Event::LostFocus) {
        printf("\033[1;33m[%s][%d] :x: focus out\033[m\n",__FUNCTION__,__LINE__);
      }

      // Quit Key ( Q )
      if (sf::Keyboard::isKeyPressed(sf::Keyboard::Q))
      {
        printf("\033[1;36m[%s][%d] :x: Quit Key \033[m\n",__FUNCTION__,__LINE__);
        window.close();
      }
    }
    window.clear(sf::Color(100,100,100,255));
    // draw things here
    window.draw(Text00);
    window.draw(Circle00);
    window.draw(Rect00);
    window.draw(Sprite00);
    window.display();
  }

  return 0;

}
