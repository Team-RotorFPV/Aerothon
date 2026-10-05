"""Does a delivery-pad QR name the place the start QR sent the aircraft?

The rulebook says only that the start QR carries "the delivery location
information" and that the aircraft must find "the QR code corresponding to"
it (4.2.4). It does not say the two carry the same string. Matching them
exactly -- as the simulator's identical payloads allowed -- fails the whole
delivery if the organisers print "Deliver to Target B" at the start and
"TARGET-B" on the pad.

So both are reduced to their IDENTIFIERS: upper-cased, split at every change
between letters and digits and at every other character, filler words
dropped (FILLER), numbers compared as numbers ("01" is "1"). A pad matches
when its identifiers equal the start's, or are contained in them -- the start
may say more ("Deliver to target B, north field") than the pad ("B").

Not the other way round: a start QR naming "B" must not match a pad "B2".
A false match delivers to the wrong pad and costs more than no match, which
the operator can still resolve with a target override (goal.md Q19).
"""

import re

# Not "A": it is the likeliest pad name of all.
FILLER = frozenset({
    "AND", "AT", "CODE", "DELIVER", "DELIVERY", "DROP", "FOR", "ID",
    "IN", "IS", "LOCATION", "NO", "NUMBER", "OF", "PAD", "PAYLOAD", "POINT",
    "QR", "SITE", "THE", "TARGET", "TO", "ZONE",
})


def identifiers(text):
    """The payload's identifying tokens, as a frozenset."""
    tokens = re.findall(r"[A-Z]+|\d+", str(text).upper())
    return frozenset(str(int(t)) if t.isdigit() else t
                     for t in tokens if t not in FILLER)


def matches(target, payload):
    """True if `payload` (a pad) names the place `target` (the start QR) does."""
    target, payload = str(target or "").strip(), str(payload or "").strip()
    if not target or not payload:
        return False
    if target == payload:
        return True
    want, got = identifiers(target), identifiers(payload)
    return bool(got) and got <= want
