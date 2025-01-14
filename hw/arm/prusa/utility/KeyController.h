/*
	KeyController.h - Wrangler for dispatching keypresses to KeyClients.

	Copyright 2020 VintagePC <https://github.com/vintagepc/>

 	This file is part of MK404.

	MK404 is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	MK404 is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with MK404.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "../utility/IScriptable.h"
#include "Scriptable.h"
#include <atomic>
#include <map>               // for map
#include <string>            // for string
#include <vector>            // for vector

class IKeyClient;

class KeyController: private Scriptable
{
	friend IKeyClient;

	public:
		static KeyController& GetController();

		// Called by the key handler to notify a key was pressed.
		inline void OnKeyPressed(unsigned char key) { m_key.store(key); };

        void OnKeyPressed_C(int keycode);

        // Keep unique clients so they can be cleaned up later.
        void AddNewClient_C(IKeyClient* src);

		// Called by the printer so key events happen "safely" on AVR cycles.
		void OnAVRCycle();

		void PrintKeys(bool bMarkdown);

		static inline void GLKeyReceiver(unsigned char key, int /*x*/, int /*y*/) { KeyController::GetController().OnKeyPressed(key); };

	protected:
		KeyController();
		~KeyController();

		// Invoked by IKeyClient to add a client.
		void AddKeyClient(IKeyClient *pClient, const unsigned char key, const std::string &strDesc);

		LineStatus ProcessAction(unsigned int iAction, const std::vector<std::string> &args) override;

	private:
		void PutNiceKeyName(unsigned char key);

		std::map<unsigned char, std::vector<IKeyClient*> > m_mClients {};
		std::map<unsigned char, std::string> m_mDescrs {};
        std::vector<IKeyClient*> m_vAllClients {};
        std::map<std::pair<int,bool>, unsigned char> m_qemu2char
        {
            { {0x009F ,true} , 'S'},
			{ {0x0091 ,false} , 'w'|0x80 },
			{ {0x009F ,false} , 's'|0x80 },
            { {0x11   ,false} , 'w'}, // shared with arrow keys for up/down
            { {0x48 ,false} , 'w'}, // shared with arrow keys for up/down
            { {0x1F   ,false} , 's'},
            { {0x50 ,false} , 's'},
            { {0x1c   ,false} ,  0xd},
			{ {0x21	  ,false} , 'f'},
			{ {0x02, false}, '1'},
			{ {0x03, false}, '2'},
			{ {0x04, false}, '3'},
			{ {0x05, false}, '4'},
			{ {0x06, false}, '5'},
			{ {0x07, false}, '6'},
			{ {0x19	  ,false} , 'p'},
			{ {0x14	  ,false} , 't'},
			{ {0x26	  ,false} , 'l'},
        };
		std::atomic_uchar m_key {0};
        bool m_bShift = false;

};

// extern void p404_keyctl_handle_key(int keycode);
