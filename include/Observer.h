#include <iostream>
#include <vector>

using namespace std;


#ifndef __OBSERVER_H__
#define __OBSERVER_H__
namespace ORB_SLAM2{

class Observer ;
class Subject ;




class Subject {
    vector < Observer * > views;
  public:
    void attach(Observer *obs);
    void update();
};


class Observer {

    Subject * model;
  public:
 
    Observer() {
    } 

    Observer(Subject *mod){
        model = mod;
        model->attach(this);
    } 

    void setSubject(Subject *mod){
	model = mod;
        model->attach(this);
    }
    virtual void update() = 0;
  protected:
    Subject *getSubject() {
        return model;
    }
};


}

#endif
