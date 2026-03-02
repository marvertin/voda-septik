#pragma once

#include <cstring>
#include <cstdint>

/**
 * Třída pro výpočet oříznutého průměru (trimmed mean)
 * Slouží k filtrování šumu v sequenci měření
 * 
 * Parametry template:
 * - BufferSize: velikost bufferu (počet hodnot k uchovávání)
 * - TrimCount: počet hodnot k odstranění z každé strany
 * 
 * Příklad:
 *   TrimmedMean<31, 5> adc_filter;  // buffer 31 prvků, odstraní 5 min a 5 max
 *   adc_filter.insert(raw_value);
 *   uint32_t avg = adc_filter.getValue();
 */
template<size_t BufferSize = 31, size_t TrimCount = 5>
class TrimmedMean
{
private:
    struct BufferEntry
    {
        int order;          // pořadí vložení
        uint32_t value;     // hodnota
    };
    
    static_assert(TrimCount < BufferSize / 2, "TrimCount musí být menší než polovina BufferSize");
    
    BufferEntry buffer[BufferSize + 2];  // buffer + 2 sentinel hodnoty
    int current_order;                    // aktuální pořadí

public:
    /**
     * Konstruktor - inicializuje buffer
     */
    TrimmedMean() : current_order(0)
    {
        // Inicializujeme buffer se sentinelem na začátku (minimální hodnota)
        for (size_t i = 0; i < BufferSize + 2; ++i)
        {
            buffer[i].order = i - 1;
            buffer[i].value = 0;
        }
        
        // Nastavíme sentinel hodnoty
        buffer[0].value = 0;
        buffer[0].order = -1;
        buffer[BufferSize + 1].value = UINT32_MAX;
        buffer[BufferSize + 1].order = -1;
    }

    /**
     * Vloží novou hodnotu do bufferu a zachová seřazení
     * 
     * @param value nová hodnota k vložení
     */
    void insert(uint32_t value)
    {
        // Najdeme index nejstarší hodnoty (podle pořadí)
        int index = 0;
        for (size_t i = 1; i < BufferSize + 1; ++i)
        {
            if (current_order == buffer[i].order)
            {
                index = i;
                break;
            }
        }

        // Vložíme novou hodnotu
        buffer[index].value = value;

        // Bubble sort - posouváme hodnotu na správné místo
        while (true)
        {
            if (buffer[index].value < buffer[index - 1].value)
            {
                // Prohodíme s levým sousedem
                BufferEntry temp = buffer[index];
                buffer[index] = buffer[index - 1];
                buffer[index - 1] = temp;
                index--;
            }
            else if (buffer[index].value > buffer[index + 1].value)
            {
                // Prohodíme s pravým sousedem
                BufferEntry temp = buffer[index];
                buffer[index] = buffer[index + 1];
                buffer[index + 1] = temp;
                index++;
            }
            else
            {
                break;  // Hodnota je na správném místě
            }
        }

        // Zvyšujeme pořadí pro další měření (modulo)
        current_order = (current_order + 1) % (int)BufferSize;
    }

    /**
     * Vrátí oříznutý průměr (removes TrimCount values from each end)
     * 
     * @return oříznutý průměr
     */
    uint32_t getValue() const
    {
        uint64_t sum = 0;
        
        // Sečteme hodnoty bez TrimCount nejmenších a TrimCount největších
        for (size_t i = 1 + TrimCount; i <= BufferSize - TrimCount; ++i)
        {
            sum += buffer[i].value;
        }
        
        return sum / (BufferSize - 2 * TrimCount);
    }

    /**
     * Vrátí velikost bufferu
     * 
     * @return velikost bufferu
     */
    size_t getBufferSize() const
    {
        return BufferSize;
    }

};
