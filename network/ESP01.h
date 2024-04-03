/**
 * ESP-01
 * 
 * @author Evgeny Darichev
 * @date 30.03.2024
 */

#ifndef _ESP01_H_INCLUDED_
#define _ESP01_H_INCLUDED_

#include <cstring>
#include <memory>
#include "../xuart.h"

// TODO: do it later, ESP-01 does not work
class ESP01 : public UARTClient
{
public:
protected:
	void send(const char *data, size_t len = -1)
	{
		if (!data)
			return;
		if (len < 0)
			len = strlen(data);
		if (!len)
			return;
	}
};

#endif // _ESP01_H_INCLUDED_
