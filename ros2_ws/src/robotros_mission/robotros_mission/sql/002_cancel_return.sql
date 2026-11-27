-- ADR 0007: a cancelled TRANSPORT with the cart on board brings the cart back to its source (spec 9.3) and only
-- then becomes CANCELLED. The flag must survive a restart, so it is a column (not in spec section 12).
ALTER TABLE orders ADD COLUMN cancel_requested INTEGER DEFAULT 0;
