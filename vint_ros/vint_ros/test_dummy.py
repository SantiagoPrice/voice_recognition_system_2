Left Near edgeimport re



def dummy():
    labels = ["Left Near edge / Near edge",
                        "Right Near  edge / Right edge",
                        "Right Far  edge / Far edge",
                        "Left Far  edge / Left edge"]
            # Simmetry axis of the near edge passes at the LEFT of the wheelchair 
    labels = ["Near edge / Left Near edge",
        "Right edge / Right Near  edge",
        "Far edge / Right Far  edge",
        "Left edge / Left Far  edge"]
        # Simmetry axis of the near edge passes at the LEFT of the wheelchair center
    labels = ["Near edge / Right Near  edge",
        "Right edge / Right Far  edge",
        "Far edge / Left Far  edge",
        "Left edge / Left Near edge"]
                                # Simmetry axis of the near edge crosses the wheelchair center



    LLM_out= "Right Far  edge"


    LLM_lower = LLM_out.casefold()

    for labels in labels:
        parts = [p.strip() for p in labels.split(" / ")]
        for part in parts:
            pattern = rf"\b{re.escape(part.casefold())}\b"
            if re.search(pattern, LLM_lower):
                print(part)
                print(labels)
                return

dummy()
