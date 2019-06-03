/**********************************************************
 *  Filename: debounceBit.h
 **********************************************************
 *
 *  Functions to debounced boolean variable.
 *
 *  Originator: Anders Sjoberg
 *  Date: 2015-11-23
 *
 *  Description: A basic misc tool.
 *
 **********************************************************/

#ifndef DEBOUNCED_BITS_H
    #define	DEBOUNCED_BITS_H
    
    #include <stdint.h>
    
typedef struct DebouncedBit_t
{
    uint8_t state;
    uint8_t counter;
    uint8_t actTimes;
    uint8_t deActTimes;
} DebouncedBit_st;

typedef uint8_t Counter_t;


// De-bounce functions
uint8_t deBoBit(DebouncedBit_st *dbb, uint8_t state);
uint8_t deBoBitHyst(DebouncedBit_st *dbb, uint8_t set, uint8_t clear);

// Counter functions
uint8_t counterCheck(Counter_t *counter);


#endif

