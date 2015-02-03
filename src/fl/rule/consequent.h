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
    class rule;
    class consequent {
    public:
        consequent();
        virtual ~consequent();
        virtual void parse(std::string expresion, rule* r);
    private:

    };
}

#endif	/* FUZZY_CONSEQUENT_H */

