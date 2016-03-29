function darap_=initializeDARAP(width, height, resource_constraint,max_items)
darap_ = darap(width,height,resource_constraint,max_items);
probability_map=get_probability_map(darap_);
imagesc(probability_map);
end