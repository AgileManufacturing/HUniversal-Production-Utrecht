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
		virtual void onBondCallback(Bond* bond, Event event) = 0;
	};
}