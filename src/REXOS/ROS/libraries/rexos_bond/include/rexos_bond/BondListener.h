#pragma once

namespace rexos_bond{
	class Bond;
	
	class BondListener {
		friend class Bond;
	public:
		enum Event{
			FORMED,
			BROKEN
		};
	protected:
		void onBondCallback(Bond* bond, Event event);
	};
}