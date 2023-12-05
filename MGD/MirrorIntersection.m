function intersectionPoint = MirrorIntersection(planePoint, planeNormal, linePoint, lineDirection )

intersectionPoint = (((planePoint - linePoint)' * planeNormal) / (lineDirection' * planeNormal)) * lineDirection + linePoint;

end
