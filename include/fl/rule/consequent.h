/* 
 * File:   consequent.h
 * Author: bpalvarado
 *
 * Created on 3 de febrero de 2015, 03:03 PM
 */

#ifndef FUZZY_CONSEQUENT_H
#define	FUZZY_CONSEQUENT_H

#include <vector>
#include <string>
#include <cmath>
#include <stdexcept>

namespace fuzzy{
    class Rule;
    class Consequent {
    public:
        Consequent();
        virtual ~Consequent();
        virtual void parse(std::string expresion, Rule* r);
    private:

    };
}

#endif	/* FUZZY_CONSEQUENT_H */

