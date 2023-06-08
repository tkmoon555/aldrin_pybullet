def get_segments(tree):
    """
    Given a PyKDL Tree object, return a list of Segment objects.
    """
    segments = []
    root_segment = tree.getRootSegment()
    segments.append(root_segment)

    def traverse(segment):
        children = tree.getSegmentChildren(segment)
        for child in children:
            segments.append(child)
            traverse(child)

    traverse(root_segment)
    return segments
