function darap_=initializeDARAP(width, height, resource_constraint)
darap_ = darap(width,height,resource_constraint);
probability_map=get_probability_map(darap_);
imagesc(probability_map);
end