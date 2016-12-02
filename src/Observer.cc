#include "Observer.h"

namespace ORB_SLAM2{


Observer::Observer(Subject *mod) {
        model = mod;
        model->attach(this);
    } 

void Observer::setSubject(Subject *mod){
	model = mod;
        model->attach(this);
}

void Subject::attach(Observer *obs) {
	views.push_back(obs);
}

void Subject::update(){
	for (int i = 0; i < views.size(); i++)
		views[i]->update();
}

}

