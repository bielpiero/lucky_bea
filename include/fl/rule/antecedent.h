/* 
 * File:   antecedent.h
 * Author: bpalvarado
 *
 * Created on 3 de febrero de 2015, 03:02 PM
 */

#ifndef FUZZY_ANTECEDENT_H
#define	FUZZY_ANTECEDENT_H

#include <vector>
#include <string>
#include <cmath>
#include <stdexcept>

namespace fuzzy{
    class Rule;

    class Antecedent {
    public:
        Antecedent();
        virtual ~Antecedent();
        virtual void parse(std::string expresion, Rule* r);
        virtual float getActivationDegree();
    private:

    };
}

#endif	/* FUZZY_ANTECEDENT_H */

